"""
symspectre_hat_flow.py - exact, symbolic version of the Spectre -> Hat collinearity flow.

This is the SymPy counterpart of spectre_hat_collinearity_flow_model.py in the same way
symspectre.py is the counterpart of spectre.py.  The flow moves the curved tile between
the Spectre (tau = 0) and the Hat (tau = 1):

    b(tau)     = a (1 + (sqrt(3) - 1) tau)          edge length b
    k(tau)     = k0 (1 - tau)                       curve strength
    omega(tau) = omega0 (1 - tau)                   turning angle of the 14th vertex P_8
    h(tau)     = a tan(omega(tau) / 2)              how far P_8 is pushed out of P_7 -> P_9

Everything is kept symbolic in (a, b, k, h) and then in (a, k0, omega0, tau); the flow
enters only through b, k and h.  The result is written out as a LaTeX document (and
compiled to PDF with pdflatex if it is installed).

The document contains
  * the flow laws,
  * the 14 edge vectors, headings and turning angles (edges 7 and 8 depend on tau),
  * the vertices P_i and a proof that the outline closes for every tau,
  * the Bezier control points, the closed form B_i(t) = P_i + s(t) d_i + k w(t) n_i,
    tangents, and the proof that the curve turns by the polygon angle at every vertex,
  * the collapse of the 14th vertex: cross product C(tau) = -2 a h(tau), its expansion at
    tau = 1, and the corner count 14 -> 13,
  * the symbolic area (Green's theorem), independent of k, as a function of tau,
  * TikZ drawings and a table of numbers for chosen tau,
  * the power-basis coefficients of every edge.

Usage:
    python3 symspectre_hat_flow.py                    # a=1, k0=0.5, omega0=30 deg
    python3 symspectre_hat_flow.py --a 1 --k0 0.5 --corner 30 --taus 0,0.25,0.5,0.75,1
    python3 symspectre_hat_flow.py --outdir out --no-pdf
    python3 symspectre_hat_flow.py --verify           # compare with the numeric flow model

Needs sympy; --verify also needs numpy (the numeric model spectre_hat_collinearity_flow_model.py
is imported from this directory).  symspectre.py provides the shared helpers.
"""

import argparse
import math
import os
import random
import sys

import sympy as sp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from symspectre import (  # noqa: E402  (shared symbols and helpers)
    N_EDGES,
    PREAMBLE,
    a,
    b,
    bernstein,
    compile_pdf,
    control_points,
    cross,
    dot,
    expand_vec,
    k,
    normal,
    s_t,
    t,
    w_t,
)
from symspectre import edge_vectors as plain_edge_vectors  # noqa: E402

# --------------------------------------------------------------------------
# Symbols and the laws of the flow
# --------------------------------------------------------------------------
h = sp.symbols("h", positive=True)  # opening of the 14th vertex, h = a tan(omega/2)
om = sp.symbols("omega", positive=True)  # omega: turning angle of P_8 (radians)
om0 = sp.symbols("omega_0", positive=True)  # omega at tau = 0
k0 = sp.symbols("k_0", real=True)  # curve strength at tau = 0
tau = sp.symbols("tau", real=True)  # flow parameter, 0 <= tau <= 1

FLAT_VERTEX = 8
EDGE_KIND = "bbaabbaaaabbaa"  # length type of each edge, as in spectremodel.py

b_tau = a * (1 + (sp.sqrt(3) - 1) * tau)
k_tau = k0 * (1 - tau)
om_tau = om0 * (1 - tau)
h_tau = a * sp.tan(om_tau / 2)
LAWS = {b: b_tau, k: k_tau, h: h_tau, om: om_tau}


def _is_zero(expr, tries=6):
    """True if expr is 0: by SymPy simplification, otherwise by exact-enough evaluation at
    random sample points of its free symbols (only used when simplification is inconclusive)."""
    e = sp.simplify(expr)
    if e == 0:
        return True
    rng = random.Random(1)
    for _ in range(tries):
        vals = {s: sp.Rational(rng.randint(2, 40), rng.randint(2, 20)) for s in e.free_symbols}
        if "omega" in {s.name for s in e.free_symbols}:
            vals[om] = sp.Rational(rng.randint(1, 30), 10)  # 0 < omega < pi
        if abs(sp.N(e.subs(vals), 40)) > 1e-25:
            return False
    return True


def _rad_over_pi(expr):
    return sp.nsimplify(sp.simplify(expr / sp.pi)) * sp.pi


# --------------------------------------------------------------------------
# The tile
# --------------------------------------------------------------------------
def edge_vectors():
    """The 14 edge vectors of the flow tile: those of spectre.py, except that edges 7 and 8
    (both (0, -a) there) become (h, -a) and (-h, -a): P_8 is pushed out by h."""
    d = list(plain_edge_vectors())
    d[7] = sp.Matrix([h, -a])
    d[8] = sp.Matrix([-h, -a])
    return d


class HatFlowSpectre:
    """All symbolic data of the flow tile, with the proofs checked by SymPy."""

    def __init__(self):
        self.d = edge_vectors()
        self.n = [normal(d) for d in self.d]
        plain = plain_edge_vectors()
        unit = {a: 1, b: 1}

        # lengths: a or b, and a / cos(omega/2) for the two edges that were opened
        self.length = [b if c == "b" else a for c in EDGE_KIND]
        self.length[7] = self.length[8] = a / sp.cos(om / 2)
        to_om = {h: a * sp.tan(om / 2)}
        for i in range(N_EDGES):
            assert _is_zero(dot(self.d[i], self.d[i]).subs(to_om) - self.length[i] ** 2), f"length, edge {i}"

        # vertices and closure
        pts = [sp.Matrix([0, 0])]
        for d in self.d:
            pts.append(expand_vec(pts[-1] + d))
        self.closure = sp.simplify(pts[-1])
        assert self.closure == sp.zeros(2, 1), "outline does not close"
        self.P = pts[:-1]
        plain_pts = [sp.Matrix([0, 0])]
        for d in plain:
            plain_pts.append(expand_vec(plain_pts[-1] + d))
        self.moved = [i for i in range(N_EDGES)
                      if sp.simplify(self.P[i] - plain_pts[i]) != sp.zeros(2, 1)]
        assert self.moved == [FLAT_VERTEX], self.moved
        assert sp.simplify(self.P[FLAT_VERTEX] - plain_pts[FLAT_VERTEX] - sp.Matrix([h, 0])) == sp.zeros(2, 1)

        # headings (radians)
        self.heading = []
        for i in range(N_EDGES):
            self.heading.append(_rad_over_pi(sp.atan2(plain[i].subs(unit)[1], plain[i].subs(unit)[0])))
        self.heading[7] = -sp.pi / 2 + om / 2
        self.heading[8] = -sp.pi / 2 - om / 2
        for i in range(N_EDGES):
            err = self.d[i] - self.length[i] * sp.Matrix([sp.cos(self.heading[i]), sp.sin(self.heading[i])])
            assert all(_is_zero(e.subs(to_om)) for e in err), f"heading, edge {i}"

        # turning angle at P_i (between d_(i-1) and d_i), radians
        self.turn = []
        for i in range(N_EDGES):
            prev, cur = plain[i - 1].subs(unit), plain[i].subs(unit)
            self.turn.append(_rad_over_pi(sp.atan2(cross(prev, cur), dot(prev, cur))))
        self.plain_turn = list(self.turn)
        self.turn[7] = self.turn[9] = -sp.pi / 3 + om / 2
        self.turn[8] = -om
        for i in range(N_EDGES):
            prev, cur = self.d[i - 1], self.d[i]
            lp, lc = self.length[i - 1], self.length[i]
            assert _is_zero((cross(prev, cur) - lp * lc * sp.sin(self.turn[i])).subs(to_om)), f"sin, vertex {i}"
            assert _is_zero((dot(prev, cur) - lp * lc * sp.cos(self.turn[i])).subs(to_om)), f"cos, vertex {i}"
        self.turn_sum = sp.simplify(sum(self.turn))
        assert self.turn_sum == -2 * sp.pi, "turning angles"
        self.changed_turns = [i for i in range(N_EDGES) if sp.simplify(self.turn[i] - self.plain_turn[i]) != 0]
        assert self.changed_turns == [7, 8, 9]

        # control points
        self.C1, self.C2 = [], []
        for i in range(N_EDGES):
            c1, c2 = control_points(self.P[i], self.d[i])
            self.C1.append(expand_vec(c1))
            self.C2.append(expand_vec(c2))

        # curves: Bernstein form, closed form, power basis
        self.B, self.power = [], []
        for i in range(N_EDGES):
            P3 = self.P[(i + 1) % N_EDGES]
            B = expand_vec(bernstein(self.P[i], self.C1[i], self.C2[i], P3))
            closed = expand_vec(self.P[i] + s_t * self.d[i] + k * w_t * self.n[i])
            assert expand_vec(B - closed) == sp.zeros(2, 1), f"closed form, edge {i}"
            self.B.append(B)
            coeffs = [sp.Matrix([sp.Poly(B[r], t).coeff_monomial(t**j) for r in range(2)]) for j in range(4)]
            self.power.append([expand_vec(c) for c in coeffs])

        # point symmetry and reversal, for a generic edge (so they hold for all 14)
        dx, dy = sp.symbols("d_x d_y", real=True)
        gd = sp.Matrix([dx, dy])
        gn = normal(gd)
        gB = gd * s_t + k * gn * w_t
        assert sp.simplify(gB + gB.subs(t, 1 - t) - gd) == sp.zeros(2, 1), "point symmetry"
        r1, r2 = control_points(gd, -gd)  # reversed edge starting at P3 = P0 + d, vector -d
        rev = bernstein(gd, r1, r2, sp.zeros(2, 1))  # P0 = 0, so B(t) = gB(t) in absolute coordinates
        assert sp.simplify(rev - gB.subs(t, 1 - t)) == sp.zeros(2, 1), "reversal"

        # tangents: B'(0) = B'(1) = M d with M = 3/2 (I - 2k R(-90 deg))
        t0 = sp.diff(gB, t).subs(t, 0)
        t1 = sp.diff(gB, t).subs(t, 1)
        assert sp.simplify(t0 - t1) == sp.zeros(2, 1)
        self.M = sp.Matrix([[sp.Rational(3, 2), -3 * k], [3 * k, sp.Rational(3, 2)]])
        assert sp.simplify(self.M * gd - t0) == sp.zeros(2, 1)
        self.scale2 = sp.Rational(9, 4) * (1 + 4 * k**2)  # M^T M = scale2 * I
        assert sp.simplify(self.M.T * self.M - self.scale2 * sp.eye(2)) == sp.zeros(2, 2)
        u, v = sp.symbols("u_x u_y", real=True), sp.symbols("v_x v_y", real=True)
        u, v = sp.Matrix(u), sp.Matrix(v)
        assert sp.simplify(cross(self.M * u, self.M * v) - self.scale2 * cross(u, v)) == 0
        assert sp.simplify(dot(self.M * u, self.M * v) - self.scale2 * dot(u, v)) == 0
        self.tan_ratio = sp.simplify(cross(gd, t0) / dot(gd, t0))
        assert self.tan_ratio == 2 * k

        # collapse of the 14th vertex: cross product of the two edges meeting at P_8
        self.C = sp.simplify(cross(self.d[FLAT_VERTEX - 1], self.d[FLAT_VERTEX]))
        assert sp.simplify(self.C + 2 * a * h) == 0
        assert _is_zero((self.C + (a**2 + h**2) * sp.sin(om)).subs(to_om)), "C = -(a^2+h^2) sin(omega)"
        self.C_tau = sp.simplify(self.C.subs(h, h_tau))
        sig = sp.symbols("sigma", positive=True)  # sigma = 1 - tau
        self.sigma = sig
        self.C_series = sp.series(-2 * a**2 * sp.tan(om0 * sig / 2), sig, 0, 4).removeO()

        # area via Green's theorem: A = 1/2 * oint (x dy - y dx)
        total, total_poly = 0, 0
        for i in range(N_EDGES):
            x, y = self.B[i]
            total += sp.integrate(sp.expand(x * sp.diff(y, t) - y * sp.diff(x, t)), (t, 0, 1))
            P3 = self.P[(i + 1) % N_EDGES]
            total_poly += cross(self.P[i], P3)
        signed = sp.simplify(total / 2)
        signed_poly = sp.simplify(total_poly / 2)
        orient = 1 if signed.subs({a: 1, b: 1, k: 0, h: 0}) > 0 else -1
        self.orientation = "counterclockwise" if orient > 0 else "clockwise"
        self.area = sp.expand(orient * signed)
        self.area_poly = sp.expand(orient * signed_poly)
        assert sp.simplify(self.area - self.area_poly) == 0, "area depends on k"
        assert sp.diff(self.area, k) == 0
        want = 2 * sp.sqrt(3) * a**2 + 3 * a * b + sp.sqrt(3) * b**2 + a * h
        assert sp.simplify(self.area - want) == 0, "area formula"
        assert self.turn_sum == (-2 * sp.pi if orient < 0 else 2 * sp.pi), "turning angles"
        self.area_tau = sp.expand(self.area.subs(LAWS))
        self.area_tau_poly = sp.expand((self.area - a * h).subs(LAWS))  # everything except the a*h triangle
        self.area0 = sp.simplify(self.area_tau.subs(tau, 0))
        self.area1 = sp.simplify(self.area_tau.subs(tau, 1))
        assert sp.simplify(self.area1 - 8 * sp.sqrt(3) * a**2) == 0

    # numeric helpers ------------------------------------------------------
    def numeric(self, av, k0v, om0v, tv):
        """P, C1, C2 (14 points each) and the area at flow parameter tv (om0v in radians)."""
        vals = {a: av, k0: k0v, om0: om0v, tau: tv}
        sub = {b: b_tau.subs(vals), k: k_tau.subs(vals), h: h_tau.subs(vals)}
        f = lambda e: float(sp.N(e.subs(sub).subs(vals)))
        P = [(f(p[0]), f(p[1])) for p in self.P]
        C1 = [(f(p[0]), f(p[1])) for p in self.C1]
        C2 = [(f(p[0]), f(p[1])) for p in self.C2]
        return P, C1, C2, f(self.area)


# --------------------------------------------------------------------------
# LaTeX helpers
# --------------------------------------------------------------------------
def tex(e):
    return sp.latex(sp.collect(sp.expand(e), [k, h, a, b]))


def pt(v):
    """Vector as a coordinate pair, e.g. \\left(x,\\ y\\right)."""
    return r"\left(" + tex(v[0]) + r",\ " + tex(v[1]) + r"\right)"


def _fmt(x):
    return "%g" % x


def tikz_frame(P, C1, C2, scale, label_vertex=True):
    """One tile (curved outline, dashed polygon) centred at the origin."""
    xs = [p[0] for p in P]
    ys = [p[1] for p in P]
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    sh = lambda p: (p[0] - cx, p[1] - cy)
    c = lambda p: "(%.5f,%.5f)" % sh(p)
    path = c(P[0])
    for i in range(N_EDGES):
        end = c(P[i + 1]) if i < N_EDGES - 1 else "cycle"
        path += " .. controls %s and %s .. %s" % (c(C1[i]), c(C2[i]), end)
    poly = " -- ".join(c(p) for p in P) + " -- cycle"
    lines = [
        r"\begin{tikzpicture}[x=%.4fcm,y=%.4fcm,line join=round]" % (scale, scale),
        r"  \draw[dashed,gray,thin] %s;" % poly,
        r"  \draw[thick,fill=blue!10,fill opacity=0.8] %s;" % path,
    ]
    if label_vertex:
        lines.append(r"  \fill[red] %s circle (1.6pt);" % c(P[FLAT_VERTEX]))
    lines.append(r"\end{tikzpicture}")
    return "\n".join(lines)


def tikz_labelled(P, C1, C2):
    """Large figure with all vertex labels (like symspectre.tikz_figure) and P_8 in red."""
    xs = [p[0] for p in P]
    ys = [p[1] for p in P]
    width = max(max(xs) - min(xs), 1e-9)
    height = max(max(ys) - min(ys), 1e-9)
    scale = min(10.0 / width, 12.0 / height)
    shoelace = sum(P[i][0] * P[(i + 1) % N_EDGES][1] - P[(i + 1) % N_EDGES][0] * P[i][1]
                   for i in range(N_EDGES))
    side = 1.0 if shoelace < 0 else -1.0

    def unit(vx, vy):
        r = math.hypot(vx, vy) or 1.0
        return vx / r, vy / r

    c = lambda p: "(%.5f,%.5f)" % p
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
        u = unit(p[0] - P[i - 1][0], p[1] - P[i - 1][1])
        v = unit(P[(i + 1) % N_EDGES][0] - p[0], P[(i + 1) % N_EDGES][1] - p[1])
        dx, dy = unit(side * (-u[1] - v[1]), side * (u[0] + v[0]))
        off = 0.55 / scale
        col = "red" if i == FLAT_VERTEX else "black"
        lines.append(r"  \fill[%s] (%.5f,%.5f) circle (1.4pt);" % (col, p[0], p[1]))
        lines.append(r"  \node[font=\scriptsize,text=%s] at (%.5f,%.5f) {$P_{%d}$};"
                     % (col, p[0] + off * dx, p[1] + off * dy, i))
    lines += [r"\end{tikzpicture}", r"\end{center}"]
    return "\n".join(lines)


def corner_count(omega_value):
    """Number of corners of the polygon: 14 unless the turn at P_8 is 0."""
    return N_EDGES - (1 if abs(omega_value) < 1e-12 else 0)


# --------------------------------------------------------------------------
# The document
# --------------------------------------------------------------------------
def build_latex(S, av, k0v, om0v, taus):
    L = []
    add = L.append
    add(PREAMBLE)
    add(r"\begin{document}")
    add(r"\title{Symbolic Spectre--Hat Collinearity Flow}")
    add(r"\author{generated by \texttt{symspectre\_hat\_flow.py} with SymPy %s}" % sp.__version__)
    add(r"\date{}")
    add(r"\maketitle")

    # ---- 1 the flow --------------------------------------------------------
    add(r"\section{The flow}")
    add(
        r"The curved tile of \texttt{symspectre.py} is moved from the Spectre ($\tau=0$) to the "
        r"Hat ($\tau=1$) by one parameter $\tau\in[0,1]$. The flow enters only through four "
        r"quantities:"
    )
    add(r"\begin{align*}")
    add(r"  b(\tau) &= a\bigl(1+(\sqrt3-1)\,\tau\bigr), & k(\tau) &= %s, \\" % sp.latex(k_tau))
    add(r"  \omega(\tau) &= %s, & h(\tau) &= %s." % (sp.latex(om_tau), sp.latex(h_tau)))
    add(r"\end{align*}")
    add(
        r"$a$ is fixed, $k$ is the curve strength, $\omega$ is the turning angle of the 14th "
        r"vertex $P_8$ in radians ($\omega_0=\pi/6$ is $30^{\circ}$, an interior angle of "
        r"$150^{\circ}$) and $h=a\tan(\omega/2)$ is how far $P_8$ is pushed out of the chord "
        r"$P_7\to P_9$. In what follows $a,b,k,h$ are symbols and the flow is recovered by "
        r"substituting the four laws above. At $\tau=1$: $b=\sqrt3\,a$, $k=0$, $\omega=h=0$, the "
        r"straight Hat; at $\tau=0$: $b=a$, $k=k_0$, $\omega=\omega_0$."
    )
    add(
        r"In the standard layout of \texttt{spectre.py} the 14 edge directions do not depend on "
        r"$a$ and $b$, and edges 7 and 8 both point along $(0,-1)$, so $P_8$ is flat for every "
        r"$(a,b)$. The flow therefore \emph{opens} $P_8$: edges 7 and 8 become $(h,-a)$ and "
        r"$(-h,-a)$. This changes the directions of two edges, keeps the chord $P_7\to P_9$ "
        r"fixed and leaves every other vertex where it was."
    )

    # ---- 2 edges -----------------------------------------------------------
    add(r"\section{Edges}")
    add(
        r"Heading is the direction of $d_i=P_{i+1}-P_i$ from the $x$-axis; the turning angle is "
        r"the exterior angle of the polygon at the start vertex $P_i$ (between $d_{i-1}$ and "
        r"$d_i$). Angles are in radians. Rows 7--9 depend on $\omega$; all others are as in "
        r"\texttt{symspectre.py}."
    )
    add(r"\begin{center}\renewcommand{\arraystretch}{1.35}")
    add(r"\begin{tabular}{@{}rcccc@{}}\toprule")
    add(r"$i$ & length & heading & $d_i$ & turn at $P_i$ \\ \midrule")
    for i in range(N_EDGES):
        add(f"{i} & ${sp.latex(S.length[i])}$ & ${sp.latex(S.heading[i])}$ & ${pt(S.d[i])}$ & "
            f"${sp.latex(S.turn[i])}$ \\\\")
    add(r"\bottomrule\end{tabular}\end{center}")
    add(
        r"SymPy verified, for every edge, that $d_i$ equals its length times the unit vector of "
        r"its heading, and for every vertex that $d_{i-1}\times d_i$ and $d_{i-1}\cdot d_i$ equal "
        r"$|d_{i-1}||d_i|\sin$ and $\cos$ of the turn (with $h=a\tan(\omega/2)$). The turns at "
        r"$P_7$ and $P_9$ change from $-\pi/3$ to $-\pi/3+\omega/2$, the turn at $P_8$ from $0$ to "
        r"$-\omega$; the three changes cancel and the turning angles sum to $%s$: the boundary is "
        r"traversed %s for every $\tau$." % (sp.latex(S.turn_sum), S.orientation)
    )

    # ---- 3 vertices --------------------------------------------------------
    add(r"\section{Vertices}")
    add(r"\begin{center}\renewcommand{\arraystretch}{1.35}")
    add(r"\begin{tabular}{@{}rl@{\qquad}rl@{}}")
    half = N_EDGES // 2
    for i in range(half):
        j = i + half
        add(f"$P_{{{i}}}$ & ${pt(S.P[i])}$ & $P_{{{j}}}$ & ${pt(S.P[j])}$ \\\\")
    add(r"\end{tabular}\end{center}")
    add(
        r"Closure check: $\sum_i d_i = %s$, i.e.\ $P_{14}=P_0$, for every $\tau$ (the two new "
        r"edges contribute $(h,-a)+(-h,-a)=(0,-2a)$, exactly what edges 7 and 8 contributed "
        r"before). Compared with the standard tile only $P_8$ moves: $P_8=P_8^{\mathrm{std}}"
        r"+(h,0)$." % pt(S.closure)
    )

    # ---- 4 control points --------------------------------------------------
    add(r"\section{B\'ezier control points}")
    add(
        r"Same rule as in \texttt{symspectre.py}: $C_{i,1}=P_i+\tfrac12 d_i-k\,n_i$, "
        r"$C_{i,2}=P_i+\tfrac12 d_i+k\,n_i$, $n_i=(d_{i,y},-d_{i,x})$, with $k=k(\tau)$. "
        r"Absolute coordinates; edge $i$ runs from $P_i$ to $P_{i+1}$."
    )
    for i in range(N_EDGES):
        add(r"\begin{align*}")
        add(f"  C_{{{i},1}} &= {pt(S.C1[i])}, \\\\")
        add(f"  C_{{{i},2}} &= {pt(S.C2[i])}.")
        add(r"\end{align*}")

    # ---- 5 closed form, tangents ------------------------------------------
    add(r"\section{Closed form and corner angles}")
    add(r"Expanding the Bernstein form of edge $i$ gives")
    add(r"\begin{equation*}")
    add(r"  B_i(t) = P_i + s(t)\,d_i + k\,w(t)\,n_i,\qquad 0\le t\le 1,")
    add(r"\end{equation*}")
    add(r"\begin{align*}")
    add(f"  s(t) &= {sp.latex(sp.expand(s_t))} = \\tfrac12\\,t\\,(3-3t+2t^2), \\\\")
    add(f"  w(t) &= {sp.latex(sp.expand(w_t))} = 3\\,t\\,(1-t)\\,(2t-1),")
    add(r"\end{align*}")
    add(
        r"verified for all 14 edges, including the two tilted ones. (Here $w(t)$ is the Bernstein "
        r"function of \texttt{symspectre.py}, not the angle $\omega$.)"
    )
    add(r"\paragraph{Properties (checked symbolically for a generic edge $d=(d_x,d_y)$).}")
    add(r"\begin{itemize}")
    add(
        r"\item \emph{Point symmetry.} $B(t)+B(1-t)=2P+d$: every edge is an S-curve, symmetric "
        r"under a half-turn about its chord midpoint; the reversed edge gives the same curve."
    )
    add(
        r"\item \emph{Tangents.} $B'(0)=B'(1)=M\,d$ with "
        r"$M=\tfrac32\begin{pmatrix}1&-2k\\2k&1\end{pmatrix}$, and "
        r"$M^{\mathsf T}M=%s\,I$. $M$ is a rotation by $\varphi=\arctan(2k)$ times a scale "
        r"(tangent angle to the chord: $\tan\varphi=%s$)." % (sp.latex(S.scale2), sp.latex(S.tan_ratio))
    )
    add(
        r"\item \emph{Corner angles.} At $P_i$ the curve arrives with tangent $M d_{i-1}$ and "
        r"leaves with tangent $M d_i$. Since $M$ multiplies cross and dot products by the same "
        r"positive factor $%s$, the angle between these tangents equals the angle between "
        r"$d_{i-1}$ and $d_i$: \emph{the curved tile has exactly the corner angles of its polygon, "
        r"for every $k$ and every $\tau$}, in particular $-\omega(\tau)$ at $P_8$."
        % sp.latex(S.scale2)
    )
    add(r"\end{itemize}")

    # ---- 6 collapse of the 14th vertex ------------------------------------
    add(r"\section{The 14th vertex}")
    add(
        r"The two edges meeting at $P_8$ are $E_A=d_7=(h,-a)$ and $E_B=d_8=(-h,-a)$, with "
        r"lengths $a/\cos(\omega/2)$. Their scalar cross product is"
    )
    add(r"\begin{equation*}")
    add(r"  C=E_A\times E_B=%s=-(a^2+h^2)\sin\omega = -2a^2\tan\frac{\omega_0(1-\tau)}{2}." % sp.latex(S.C))
    add(r"\end{equation*}")
    add(
        r"It is non-zero for $\tau<1$, where $P_8$ is a corner with turning angle $-\omega(\tau)$ "
        r"(both for the polygon and for the curve), and vanishes exactly at $\tau=1$, where the "
        r"two edges become parallel and merge into one straight side of length $2a$. Near $\tau=1$, "
        r"with $\sigma=1-\tau$,"
    )
    add(r"\begin{equation*}")
    add(r"  C = %s + O(\sigma^{5}),\qquad\text{so}\quad C\approx -a^2\omega_0\,(1-\tau)." % sp.latex(S.C_series))
    add(r"\end{equation*}")
    add(
        r"It shrinks monotonically but is not linear in $\tau$. The number of corners is 14 for "
        r"$0\le\tau<1$ and 13 at $\tau=1$ (provided $\omega_0>0$). Without the opening "
        r"($\omega_0=0$) $P_8$ is flat for all $\tau$."
    )

    # ---- 7 area ------------------------------------------------------------
    add(r"\section{Area}")
    add(
        r"By Green's theorem $A=\tfrac12\oint (x\,dy-y\,dx)$ on the polynomial curves $B_i$ "
        r"(boundary traversed %s):" % S.orientation
    )
    add(r"\begin{equation*}")
    add(f"  A = {sp.latex(S.area)} ,")
    add(r"\end{equation*}")
    add(
        r"the area of the straight 14-gon, with $\partial A/\partial k=0$: the lobes of the "
        r"S-curves cancel. The term $a\,h$ is the triangle added by opening $P_8$. As a function "
        r"of the flow parameter,"
    )
    add(r"\begin{equation*}")
    add(r"  A(\tau) = %s + a^2\tan\frac{\omega_0(1-\tau)}{2}," % sp.latex(sp.collect(S.area_tau_poly, a)))
    add(r"\end{equation*}")
    add(
        r"with $A(0)=%s$ and $A(1)=%s$." % (sp.latex(sp.factor(S.area0)), sp.latex(S.area1))
    )

    # ---- 8 figures ---------------------------------------------------------
    frames = [S.numeric(av, k0v, om0v, tv) for tv in taus]
    add(r"\section{Figures}")
    add(tikz_labelled(*frames[0][:3]))
    add(
        r"\begin{center}\small The tile at $\tau=%s$ ($a=%s$, $k_0=%s$, $\omega_0=%s$ rad): "
        r"curved outline, dashed polygon; the opened vertex $P_8$ is red.\end{center}"
        % (_fmt(taus[0]), _fmt(av), _fmt(k0v), _fmt(om0v))
    )
    allx = [p[0] for fr in frames for p in fr[0]]
    ally = [p[1] for fr in frames for p in fr[0]]
    wmax = max(max(p[0] for p in fr[0]) - min(p[0] for p in fr[0]) for fr in frames)
    hmax = max(max(p[1] for p in fr[0]) - min(p[1] for p in fr[0]) for fr in frames)
    n = len(frames)
    scale = min(14.0 / (n * wmax) , 3.4 / hmax)  # all frames side by side in about 14 cm
    add(r"\begin{center}")
    for tv, fr in zip(taus, frames):
        add(r"\begin{minipage}{%.3f\linewidth}\centering" % (0.96 / n))
        add(tikz_frame(fr[0], fr[1], fr[2], scale))
        add(r"\\[-0.2em]{\scriptsize $\tau=%s$}" % _fmt(tv))
        add(r"\end{minipage}")
    add(r"\end{center}")
    add(r"\begin{center}\small The flow from the Spectre to the Hat. Dashed: the polygon.\end{center}")

    add(r"\begin{center}\renewcommand{\arraystretch}{1.25}")
    add(r"\begin{tabular}{@{}rrrrrrrr@{}}\toprule")
    add(r"$\tau$ & $b$ & $k$ & $\omega$ (deg) & $h$ & $|d_7|=|d_8|$ & corners & area \\ \midrule")
    for tv, fr in zip(taus, frames):
        bv = float(sp.N(b_tau.subs({a: av, tau: tv})))
        kv = k0v * (1 - tv)
        wv = om0v * (1 - tv)
        hv = av * math.tan(wv / 2)
        ln = av / math.cos(wv / 2)
        add(r"%s & %.4f & %.4f & %.2f & %.4f & %.4f & %d & %.4f \\"
            % (_fmt(tv), bv, kv, math.degrees(wv), hv, ln, corner_count(wv), fr[3]))
    add(r"\bottomrule\end{tabular}\end{center}")

    # ---- appendix ----------------------------------------------------------
    add(r"\appendix")
    add(r"\section{Power-basis form}")
    add(
        r"Each edge as $B_i(t)=P_i+c_{i,1}\,t+c_{i,2}\,t^2+c_{i,3}\,t^3$ "
        r"(Horner evaluation), in terms of $a,b,k,h$."
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


# --------------------------------------------------------------------------
# Optional numeric cross-check against spectre_hat_collinearity_flow_model.py
# --------------------------------------------------------------------------
def verify_against_numeric(S, av, k0v, om0v, taus, samples=50):
    """Compare the symbolic curves with the numeric SpectreHatFlow (needs numpy)."""
    try:
        import numpy as np

        from spectre_hat_collinearity_flow_model import SpectreHatFlow
    except ImportError as exc:
        print("--verify needs numpy and spectre_hat_collinearity_flow_model.py: %s" % exc)
        return False

    flow = SpectreHatFlow(0.0, k0=k0v, a=av, corner_deg=math.degrees(om0v))
    tt = np.linspace(0.0, 1.0, samples)
    worst = 0.0
    for tv in taus:
        flow.tau = tv
        num = flow.tile.sample_edges(samples - 1)  # shape (14, samples, 2)
        sub = {a: av, b: b_tau.subs({a: av, tau: tv}), k: k_tau.subs({k0: k0v, tau: tv}),
               h: h_tau.subs({a: av, om0: om0v, tau: tv})}
        for i in range(N_EDGES):
            fx = sp.lambdify(t, S.B[i][0].subs(sub), "numpy")
            fy = sp.lambdify(t, S.B[i][1].subs(sub), "numpy")
            sx = np.asarray(fx(tt), dtype=float) * np.ones_like(tt)
            sy = np.asarray(fy(tt), dtype=float) * np.ones_like(tt)
            worst = max(worst, float(np.max(np.abs(sx - num[i, :, 0]))), float(np.max(np.abs(sy - num[i, :, 1]))))
        area_sym = float(sp.N(S.area.subs(sub)))
        worst = max(worst, abs(area_sym - flow.tile.area()))
    ok = worst < 1e-9
    print("verify vs spectre_hat_collinearity_flow_model.py (tau in %s): max deviation %.2e -> %s"
          % (list(taus), worst, "OK" if ok else "MISMATCH"))
    return ok


# --------------------------------------------------------------------------
def _parse_taus(text):
    try:
        taus = [float(x) for x in text.split(",") if x.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError("--taus expects comma separated numbers, got %r" % text)
    if not taus or any(not (0.0 <= x <= 1.0) for x in taus):
        raise argparse.ArgumentTypeError("--taus must be numbers in [0, 1], got %r" % text)
    return taus


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Symbolic Spectre -> Hat collinearity flow -> LaTeX/PDF (SymPy)."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge a, for the figures (default 1)")
    p.add_argument("--k0", type=float, default=0.5, help="curve strength at tau = 0 (default 0.5)")
    p.add_argument("--corner", type=float, default=30.0, metavar="DEG",
                   help="turning angle omega_0 of the 14th vertex at tau = 0, degrees in [0, 180) (default 30)")
    p.add_argument("--taus", type=_parse_taus, default=[0.0, 0.25, 0.5, 0.75, 1.0],
                   help="comma separated tau values for the figures (default 0,0.25,0.5,0.75,1)")
    p.add_argument("--outdir", default=".", help="output directory (default: current)")
    p.add_argument("--name", default="symspectre_hat_flow", help="output file stem (default: symspectre_hat_flow)")
    p.add_argument("--no-pdf", action="store_true", help="only write the .tex file")
    p.add_argument("--verify", action="store_true",
                   help="cross-check against spectre_hat_collinearity_flow_model.py (needs numpy)")
    args = p.parse_args(argv)
    if not (math.isfinite(args.a) and args.a > 0 and math.isfinite(args.k0)):
        p.error("--a must be positive and finite, --k0 finite")
    if not (0.0 <= args.corner < 180.0):
        p.error("--corner must be in [0, 180) degrees")
    if args.corner == 0.0:
        p.error("--corner 0 switches the opening off (P_8 flat for all tau); the symbolic model "
                "assumes omega_0 > 0, use a small positive value")
    om0v = math.radians(args.corner)

    S = HatFlowSpectre()
    print("area       A = %s   (independent of k)" % S.area)
    print("area(tau)  A = %s + a**2*tan(omega_0*(1 - tau)/2)" % sp.collect(S.area_tau_poly, a))
    print("cross      C = %s = %s" % (S.C, S.C_tau))
    print("turning angles changed at vertices %s, sum %s" % (S.changed_turns, S.turn_sum))

    if args.verify and not verify_against_numeric(S, args.a, args.k0, om0v, args.taus):
        return 1

    source = build_latex(S, args.a, args.k0, om0v, args.taus)
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
