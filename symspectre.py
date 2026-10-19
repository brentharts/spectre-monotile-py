"""
symspectre.py - exact, symbolic version of the *curved* Spectre monotile.

This is the SymPy counterpart of spectre.py: the same 14-sided tile with edge
lengths ``a`` and ``b``, where every straight edge is replaced by the S-shaped
cubic Bezier curve that spectre.py draws.  Everything is kept symbolic in
(a, b, k), where ``k`` is the curve strength, and written out as a LaTeX
document (and compiled to PDF with pdflatex if it is installed).

The document contains
  * the 14 edge vectors, headings and turning angles,
  * the vertices P_i and a proof that the outline closes,
  * the Bezier control points of every edge (absolute coordinates),
  * the closed-form parametrisation B_i(t) = P_i + s(t) d_i + k w(t) n_i,
    and the power-basis coefficients of every edge,
  * the symbolic area (Green's theorem) and the proof that it does not
    depend on the curve strength,
  * a TikZ drawing for numeric values of a, b and k.

Usage:
    python3 symspectre.py                       # a=1, b=1, k=0.5 for the figure
    python3 symspectre.py --a 1 --b 1 --curve_strength 0.5
    python3 symspectre.py --outdir out --no-pdf
    python3 symspectre.py --verify              # compare with spectre.py numerically
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import tempfile

import sympy as sp

# --------------------------------------------------------------------------
# Symbols
# --------------------------------------------------------------------------
a, b = sp.symbols("a b", positive=True)  # the two edge lengths
k = sp.symbols("k", real=True)  # curve strength (curve_strength in spectre.py)
t = sp.symbols("t", real=True)  # Bezier parameter, 0 <= t <= 1

N_EDGES = 14


# --------------------------------------------------------------------------
# The tile
# --------------------------------------------------------------------------
def edge_vectors():
    """The 14 edge vectors of the tile, exactly as in spectre.generate_monotile
    but with cos(60 deg) = 1/2 and sin(60 deg) = sqrt(3)/2 kept exact."""
    c, s = sp.Rational(1, 2), sp.sqrt(3) / 2
    raw = [
        (c * b, s * b),
        (b, 0),
        (0, a),
        (s * a, c * a),
        (c * b, -s * b),
        (-c * b, -s * b),
        (s * a, -c * a),
        (0, -a),
        (0, -a),
        (-s * a, -c * a),
        (-c * b, s * b),
        (-b, 0),
        (0, a),
        (-s * a, c * a),
    ]
    return [sp.Matrix(v) for v in raw]


def normal(d):
    """Un-normalised edge normal used by spectre.py: (dy, -dx)."""
    return sp.Matrix([d[1], -d[0]])


def control_points(P0, d):
    """Absolute control points of the Bezier curve for the edge P0 -> P0 + d.
    Same rule as spectre.calculate_control_points: P0 + d/2 -/+ k n."""
    n = normal(d)
    return P0 + d / 2 - k * n, P0 + d / 2 + k * n


def bernstein(P0, C1, C2, P3):
    """Cubic Bezier in Bernstein form."""
    return (
        (1 - t) ** 3 * P0
        + 3 * (1 - t) ** 2 * t * C1
        + 3 * (1 - t) * t**2 * C2
        + t**3 * P3
    )


# Closed form: B(t) = P0 + s(t) d + k w(t) n
s_t = t * (3 - 3 * t + 2 * t**2) / 2
w_t = 3 * t * (1 - t) * (2 * t - 1)


def cross(u, v):
    return u[0] * v[1] - u[1] * v[0]


def dot(u, v):
    return u[0] * v[0] + u[1] * v[1]


def expand_vec(v):
    return v.applyfunc(sp.expand)


class CurvedSpectre:
    """All symbolic data of the curved tile, with the proofs checked by SymPy."""

    def __init__(self):
        self.d = edge_vectors()
        self.n = [normal(d) for d in self.d]

        # vertices and closure
        pts = [sp.Matrix([0, 0])]
        for d in self.d:
            pts.append(expand_vec(pts[-1] + d))
        self.closure = sp.simplify(pts[-1])
        assert self.closure == sp.zeros(2, 1), "outline does not close"
        self.P = pts[:-1]

        # headings and turning angles (a, b > 0, so use a = b = 1)
        unit = {a: 1, b: 1}
        self.heading = [
            sp.nsimplify(sp.atan2(d[1].subs(unit), d[0].subs(unit)) / sp.pi * 180)
            for d in self.d
        ]
        self.turn = []
        for i in range(N_EDGES):
            prev, cur = self.d[i - 1].subs(unit), self.d[i].subs(unit)
            ang = sp.atan2(cross(prev, cur), dot(prev, cur)) / sp.pi * 180
            self.turn.append(sp.nsimplify(ang))  # turn at vertex P_i
        self.turn_sum = sum(self.turn)

        # control points
        self.C1, self.C2 = [], []
        for i in range(N_EDGES):
            c1, c2 = control_points(self.P[i], self.d[i])
            self.C1.append(expand_vec(c1))
            self.C2.append(expand_vec(c2))

        # curves: Bernstein form, closed form, power basis
        self.B = []
        self.power = []  # per edge: [c0, c1, c2, c3] with B = sum c_j t^j
        for i in range(N_EDGES):
            P3 = self.P[(i + 1) % N_EDGES]
            B = expand_vec(bernstein(self.P[i], self.C1[i], self.C2[i], P3))
            closed = expand_vec(self.P[i] + s_t * self.d[i] + k * w_t * self.n[i])
            assert sp.simplify(B - closed) == sp.zeros(2, 1), f"closed form, edge {i}"
            self.B.append(B)
            coeffs = [
                sp.Matrix([sp.Poly(B[r], t).coeff_monomial(t**j) for r in range(2)])
                for j in range(4)
            ]
            self.power.append([expand_vec(c) for c in coeffs])

        # S-curves are point-symmetric about the chord midpoint:
        # B(t) + B(1 - t) = 2 P0 + d
        for i in range(N_EDGES):
            resid = (self.B[i] + self.B[i].subs(t, 1 - t)) - (2 * self.P[i] + self.d[i])
            assert sp.simplify(resid) == sp.zeros(2, 1), f"symmetry, edge {i}"

        # reversing an edge (start at P3, vector -d) reproduces the same curve
        for i in range(N_EDGES):
            P3 = self.P[(i + 1) % N_EDGES]
            r1, r2 = control_points(P3, -self.d[i])
            rev = bernstein(P3, r1, r2, self.P[i])
            assert sp.simplify(rev - self.B[i].subs(t, 1 - t)) == sp.zeros(2, 1), (
                f"reversal, edge {i}"
            )

        # tangents at the end points of a generic edge d = (dx, dy)
        dx, dy = sp.symbols("d_x d_y", real=True)
        gd = sp.Matrix([dx, dy])
        gn = normal(gd)
        gB = gd * s_t + k * gn * w_t
        t0 = sp.diff(gB, t).subs(t, 0)
        t1 = sp.diff(gB, t).subs(t, 1)
        assert sp.simplify(t0 - t1) == sp.zeros(2, 1)
        self.tangent = sp.simplify(t0)
        self.tan_ratio = sp.simplify(cross(gd, t0) / dot(gd, t0))  # tan(angle to chord)
        assert self.tan_ratio == 2 * k

        # area via Green's theorem: A = 1/2 * oint (x dy - y dx)
        total = 0
        total_poly = 0
        for i in range(N_EDGES):
            x, y = self.B[i]
            total += sp.integrate(sp.expand(x * sp.diff(y, t) - y * sp.diff(x, t)), (t, 0, 1))
            P3 = self.P[(i + 1) % N_EDGES]
            total_poly += cross(self.P[i], P3)
        signed = sp.simplify(total / 2)
        signed_poly = sp.simplify(total_poly / 2)
        orient = 1 if signed.subs({a: 1, b: 1, k: 0}) > 0 else -1
        self.orientation = "counterclockwise" if orient > 0 else "clockwise"
        self.area = sp.factor(sp.expand(orient * signed))
        self.area_poly = sp.factor(sp.expand(orient * signed_poly))
        assert sp.simplify(self.area - self.area_poly) == 0, "area depends on k"
        assert sp.diff(self.area, k) == 0
        assert self.turn_sum == (360 if orient > 0 else -360), "turning angles"

    # numeric helpers ------------------------------------------------------
    def numeric(self, av, bv, kv):
        vals = {a: av, b: bv, k: kv}
        f = lambda e: float(sp.N(e.subs(vals)))
        P = [(f(p[0]), f(p[1])) for p in self.P]
        C1 = [(f(p[0]), f(p[1])) for p in self.C1]
        C2 = [(f(p[0]), f(p[1])) for p in self.C2]
        return P, C1, C2, f(self.area)


# --------------------------------------------------------------------------
# LaTeX helpers
# --------------------------------------------------------------------------
def tex(e):
    return sp.latex(sp.collect(sp.expand(e), [k, a, b]))


def pt(v):
    """Vector as a coordinate pair, e.g. \\left(x,\\ y\\right)."""
    return r"\left(" + tex(v[0]) + r",\ " + tex(v[1]) + r"\right)"


def deg(v):
    return sp.latex(v) + r"^{\circ}"


PREAMBLE = r"""\documentclass[11pt]{article}
\usepackage[a4paper, margin=0.9in]{geometry}
\usepackage{amsmath, amssymb}
\usepackage{booktabs, longtable}
\usepackage{tikz}
\usepackage[hidelinks]{hyperref}
\allowdisplaybreaks
\setlength{\parindent}{0pt}
\setlength{\parskip}{0.6em}
\newcommand{\cross}{\mathbin{\times}}
"""


def build_latex(S, av, bv, kv):
    L = []
    add = L.append
    add(PREAMBLE)
    add(r"\begin{document}")
    add(r"\title{Symbolic Curved Spectre Monotile}")
    add(r"\author{generated by \texttt{symspectre.py} with SymPy %s}" % sp.__version__)
    add(r"\date{}")
    add(r"\maketitle")

    # ---- 1 construction ---------------------------------------------------
    add(r"\section{Construction}")
    add(
        r"The Spectre tile is a 14-gon whose edges have length $a$ or $b$. "
        r"Its vertices are $P_0,\dots,P_{13}$ and edge $i$ is the vector "
        r"$d_i=P_{i+1}-P_i$ (indices mod 14), with $P_0=(0,0)$. "
        r"In the curved version each edge is replaced by the cubic B\'ezier curve "
        r"with control points"
    )
    add(r"\begin{align*}")
    add(r"  C_{i,1} &= P_i + \tfrac12 d_i - k\,n_i, &")
    add(r"  C_{i,2} &= P_i + \tfrac12 d_i + k\,n_i, &")
    add(r"  n_i &= (d_{i,y},\,-d_{i,x}),")
    add(r"\end{align*}")
    add(
        r"where $k$ is the curve strength (\texttt{curve\_strength} in "
        r"\texttt{spectre.py}). Note that $n_i$ is not normalised, so the "
        r"bulge of an edge scales with its length. For $k=0$ the curve "
        r"degenerates to the straight edge. All results below are exact in $a$, $b$ and $k$."
    )

    # ---- 2 edges ----------------------------------------------------------
    add(r"\section{Edges}")
    add(
        r"Heading is the direction of $d_i$ measured from the $x$-axis; the "
        r"turning angle is the exterior angle of the polygon at the start "
        r"vertex $P_i$ (between $d_{i-1}$ and $d_i$)."
    )
    add(r"\begin{center}\renewcommand{\arraystretch}{1.35}")
    add(r"\begin{tabular}{@{}rcccc@{}}\toprule")
    add(r"$i$ & length & heading & $d_i$ & turn at $P_i$ \\ \midrule")
    for i in range(N_EDGES):
        length = "a" if S.d[i].has(a) else "b"
        add(
            f"{i} & ${length}$ & ${deg(S.heading[i])}$ & ${pt(S.d[i])}$ & ${deg(S.turn[i])}$ \\\\"
        )
    add(r"\bottomrule\end{tabular}\end{center}")
    add(
        r"The turning angles sum to $%s$, one full turn: the boundary is traversed %s "
        r"(a simple closed polygon turns by $\pm360^{\circ}$)."
        % (deg(S.turn_sum), S.orientation)
    )

    # ---- 3 vertices -------------------------------------------------------
    add(r"\section{Vertices}")
    add(r"\begin{center}\renewcommand{\arraystretch}{1.35}")
    add(r"\begin{tabular}{@{}rl@{\qquad}rl@{}}")
    half = N_EDGES // 2
    for i in range(half):
        j = i + half
        add(f"$P_{{{i}}}$ & ${pt(S.P[i])}$ & $P_{{{j}}}$ & ${pt(S.P[j])}$ \\\\")
    add(r"\end{tabular}\end{center}")
    add(
        r"Closure check: $\sum_i d_i = %s$, i.e.\ $P_{14}=P_0$." % pt(S.closure)
    )

    # ---- 4 control points -------------------------------------------------
    add(r"\section{B\'ezier control points}")
    add(r"Absolute coordinates; edge $i$ runs from $P_i$ to $P_{i+1}$.")
    for i in range(N_EDGES):
        add(r"\begin{align*}")
        add(f"  C_{{{i},1}} &= {pt(S.C1[i])}, \\\\")
        add(f"  C_{{{i},2}} &= {pt(S.C2[i])}.")
        add(r"\end{align*}")

    # ---- 5 closed form ----------------------------------------------------
    add(r"\section{Closed form of the curves}")
    add(r"Expanding the Bernstein form of edge $i$ gives")
    add(r"\begin{equation*}")
    add(r"  B_i(t) = P_i + s(t)\,d_i + k\,w(t)\,n_i,\qquad 0\le t\le 1,")
    add(r"\end{equation*}")
    add(r"\begin{align*}")
    add(f"  s(t) &= {sp.latex(sp.expand(s_t))} = \\tfrac12\\,t\\,(3-3t+2t^2), \\\\")
    add(f"  w(t) &= {sp.latex(sp.expand(w_t))} = 3\\,t\\,(1-t)\\,(2t-1).")
    add(r"\end{align*}")
    add(
        r"SymPy verified this identity for all 14 edges "
        r"(Bernstein form minus closed form simplifies to $0$)."
    )
    add(r"\paragraph{Properties (all checked symbolically).}")
    add(r"\begin{itemize}")
    add(
        r"\item \emph{Point symmetry.} $s(t)+s(1-t)=1$ and $w(1-t)=-w(t)$, hence "
        r"$B_i(t)+B_i(1-t)=2P_i+d_i$: every edge is an S-curve, symmetric under a "
        r"half-turn about the midpoint of its chord. Applying the construction to the "
        r"reversed edge ($P_{i+1}\to P_i$, vector $-d_i$) gives the same curve, so a "
        r"rotated copy of the tile meeting this edge from the other side sees the same shape."
    )
    add(
        r"\item \emph{Tangents.} For an edge $d=(d_x,d_y)$, "
        r"$B'(0)=B'(1)=%s$. The tangent therefore makes the angle "
        r"$\varphi=\arctan(2k)$ with the chord at \emph{both} ends "
        r"($\tan\varphi=\frac{d\times B'(0)}{d\cdot B'(0)}=%s$)."
        % (pt(S.tangent), sp.latex(S.tan_ratio))
    )
    add(
        r"\item \emph{Corner angles.} At each vertex the incoming and outgoing "
        r"tangents are both rotated by $\varphi$ relative to their chords, so the "
        r"corner angles of the curved tile equal those of the polygon (table above)."
    )
    add(r"\end{itemize}")

    # ---- 6 area -----------------------------------------------------------
    add(r"\section{Area}")
    add(
        r"By Green's theorem $A=\tfrac12\oint (x\,dy-y\,dx)$, evaluated edge by edge "
        r"on the polynomial curves $B_i$ (the boundary is traversed %s):" % S.orientation
    )
    add(r"\begin{equation*}")
    add(f"  A = {sp.latex(S.area)} .")
    add(r"\end{equation*}")
    add(
        r"This equals the area of the straight 14-gon obtained with $k=0$, and "
        r"$\partial A/\partial k=0$: the lobes cut off and added by the S-shaped "
        r"edges cancel, so the curve strength changes the shape but not the area."
    )

    # ---- 7 figure ---------------------------------------------------------
    P, C1, C2, area_num = S.numeric(av, bv, kv)
    add(r"\section{Figure}")
    add(tikz_figure(P, C1, C2))
    add(
        r"\begin{center}\small Curved Spectre tile for $a=%s$, $b=%s$, $k=%s$ "
        r"(area $%.6f$). Dashed: the straight polygon.\end{center}"
        % (_fmt(av), _fmt(bv), _fmt(kv), area_num)
    )

    # ---- appendix ---------------------------------------------------------
    add(r"\appendix")
    add(r"\section{Power-basis form}")
    add(
        r"Each edge as $B_i(t)=P_i+c_{i,1}\,t+c_{i,2}\,t^2+c_{i,3}\,t^3$ "
        r"(convenient for evaluation with Horner's scheme)."
    )
    for i in range(N_EDGES):
        c = S.power[i]
        add(r"\begin{align*}")
        add(f"  c_{{{i},1}} &= {pt(c[1])}, \\\\")
        add(f"  c_{{{i},2}} &= {pt(c[2])}, \\\\")
        add(f"  c_{{{i},3}} &= {pt(c[3])}.")
        add(r"\end{align*}")

    add(r"\end{document}")
    return "\n".join(L) + "\n"


def _fmt(x):
    return ("%g" % x)


def tikz_figure(P, C1, C2):
    xs = [p[0] for p in P]
    ys = [p[1] for p in P]
    width = max(max(xs) - min(xs), 1e-9)
    height = max(max(ys) - min(ys), 1e-9)
    scale = min(11.0 / width, 13.0 / height)
    # orientation of the polygon, to point label offsets outwards
    shoelace = sum(P[i][0] * P[(i + 1) % N_EDGES][1] - P[(i + 1) % N_EDGES][0] * P[i][1]
                   for i in range(N_EDGES))
    side = 1.0 if shoelace < 0 else -1.0  # clockwise: outside is the left normal

    def unit(vx, vy):
        r = math.hypot(vx, vy) or 1.0
        return vx / r, vy / r

    def c(p):
        return "(%.5f,%.5f)" % p

    path = c(P[0])
    for i in range(N_EDGES):
        end = c(P[i + 1]) if i < N_EDGES - 1 else "cycle"
        path += " .. controls %s and %s .. %s" % (c(C1[i]), c(C2[i]), end)

    poly = " -- ".join(c(p) for p in P) + " -- cycle"

    lines = [
        r"\begin{center}",
        r"\begin{tikzpicture}[x=%.4fcm,y=%.4fcm,line join=round]" % (scale, scale),
        r"  \draw[dashed,gray] %s;" % poly,
        r"  \draw[thick,fill=blue!10,fill opacity=0.8] %s;" % path,
    ]
    for i, p in enumerate(P):
        # outward bisector of the two adjacent edge normals (works at concave corners too)
        u = unit(p[0] - P[i - 1][0], p[1] - P[i - 1][1])
        v = unit(P[(i + 1) % N_EDGES][0] - p[0], P[(i + 1) % N_EDGES][1] - p[1])
        nx = side * (-u[1] - v[1])
        ny = side * (u[0] + v[0])
        dx, dy = unit(nx, ny)
        off = 0.55 / scale
        lx, ly = p[0] + off * dx, p[1] + off * dy
        lines.append(r"  \fill (%.5f,%.5f) circle (1.2pt);" % p)
        lines.append(
            r"  \node[font=\scriptsize] at (%.5f,%.5f) {$P_{%d}$};" % (lx, ly, i)
        )
    lines += [r"\end{tikzpicture}", r"\end{center}"]
    return "\n".join(lines)


# --------------------------------------------------------------------------
# PDF output
# --------------------------------------------------------------------------
def compile_pdf(tex_source, outdir, name):
    """Compile in a scratch directory; keep only <name>.tex and <name>.pdf."""
    os.makedirs(outdir, exist_ok=True)
    tex_path = os.path.join(outdir, name + ".tex")
    with open(tex_path, "w", encoding="utf-8") as fh:
        fh.write(tex_source)
    if shutil.which("pdflatex") is None:
        print("pdflatex not found; wrote %s only." % tex_path)
        return tex_path, None
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, name + ".tex")
        shutil.copy(tex_path, src)
        proc = subprocess.run(
            ["pdflatex", "-interaction=nonstopmode", "-halt-on-error", name + ".tex"],
            cwd=tmp,
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            print(proc.stdout[-2000:])
            raise SystemExit("pdflatex failed (see output above)")
        pdf_path = os.path.join(outdir, name + ".pdf")
        shutil.copy(os.path.join(tmp, name + ".pdf"), pdf_path)
        overfull = [ln for ln in proc.stdout.splitlines() if ln.startswith("Overfull")]
        if overfull:
            print("warning: %d overfull box(es) in the LaTeX output" % len(overfull))
    return tex_path, pdf_path


# --------------------------------------------------------------------------
# Optional numeric cross-check against spectre.py
# --------------------------------------------------------------------------
def verify_against_numeric(S, av, bv, kv, samples=50):
    """Compare the symbolic curves with spectre.generate_monotile (needs numpy;
    spectre.py falls back to bezierpy when `bezier` is not installed)."""
    try:
        import numpy as np

        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import spectre
    except ImportError as exc:
        print("--verify needs numpy (pip install numpy): %s" % exc)
        return False

    xs, ys = spectre.generate_monotile(av, bv, kv)
    tt = np.linspace(0.0, 1.0, samples)
    vals = {a: av, b: bv, k: kv}
    worst = 0.0
    for i in range(N_EDGES):
        fx = sp.lambdify(t, S.B[i][0].subs(vals), "numpy")
        fy = sp.lambdify(t, S.B[i][1].subs(vals), "numpy")
        sx = np.asarray(fx(tt), dtype=float) * np.ones_like(tt)
        sy = np.asarray(fy(tt), dtype=float) * np.ones_like(tt)
        lo = 1 + i * samples
        err = max(
            np.max(np.abs(sx - np.asarray(xs[lo : lo + samples]))),
            np.max(np.abs(sy - np.asarray(ys[lo : lo + samples]))),
        )
        worst = max(worst, float(err))
    ok = worst < 1e-9
    print("verify vs spectre.py: max deviation %.2e -> %s" % (worst, "OK" if ok else "MISMATCH"))
    return ok


# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Symbolic curved Spectre monotile -> LaTeX/PDF (SymPy)."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge a, for the figure (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge b, for the figure (default 1)")
    p.add_argument(
        "--curve_strength",
        type=float,
        default=0.5,
        help="curve strength k, for the figure (default 0.5)",
    )
    p.add_argument("--outdir", default=".", help="output directory (default: current)")
    p.add_argument("--name", default="symspectre", help="output file stem (default: symspectre)")
    p.add_argument("--no-pdf", action="store_true", help="only write the .tex file")
    p.add_argument(
        "--verify",
        action="store_true",
        help="cross-check the symbolic curves against spectre.py (needs numpy)",
    )
    args = p.parse_args(argv)

    S = CurvedSpectre()
    print("area       A = %s   (independent of k)" % S.area)
    print("tangent angle to chord: arctan(%s)" % S.tan_ratio)

    if args.verify and not verify_against_numeric(S, args.a, args.b, args.curve_strength):
        return 1

    source = build_latex(S, args.a, args.b, args.curve_strength)
    if args.no_pdf:
        os.makedirs(args.outdir, exist_ok=True)
        path = os.path.join(args.outdir, args.name + ".tex")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(source)
        print("wrote", path)
    else:
        tex_path, pdf_path = compile_pdf(source, args.outdir, args.name)
        print("wrote", tex_path)
        if pdf_path:
            print("wrote", pdf_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
