"""
_tb.py - small tight-binding helpers for the models (numpy and scipy only).

A *graph* here is a set of points and bonds; a tight-binding model puts a number on every bond (the hopping t)
and optionally on every site (the onsite energy).  The matrix is  H[i, j] = t_ij  for a bond, H[i, i] = onsite_i.
(The sign convention is +t, as in pyqula; -t flips the spectrum, and for a bipartite graph the spectrum is symmetric
about 0 anyway.)

    from _tb import patch_graph, tight_binding, spectrum, ipr

    g = patch_graph(3, a=1, b=1)                 # the level-3 Spectre patch: 3734 sites, 4292 bonds
    H = tight_binding(g, hopping="inverse")      # t = 1 / bond length
    w, v = spectrum(H)                           # dense, exact diagonalisation of a finite patch
    print(w.max(), (ipr(v) * len(w)).mean())

Exact facts (all in the tests): V - E + F = 1 for a patch; the bond graphs of Hat (1, sqrt3), Spectre (1, 1) and Turtle
(sqrt3, 1) are the same graph, so with unit hopping their spectra coincide; a bipartite graph has a spectrum symmetric
about 0; sum E^2 = sum of H_ij^2 (the trace identity).  Anything about the spectra beyond these is a measurement of a
*finite patch with a boundary*, not of the infinite tiling.

The pieces follow the ideas of ``extras/ql_tilings.py`` and ``ql_tiling_spectra.py`` of the quantum-lattice repository,
but the code is written here from the definitions (GPL code is not copied; see models/README.md).  The planar graph
comes from ``spectre_patch.SpectrePatch`` of this repository, so nothing else is needed.

    python3 models/_tb.py                  tests, then the table of the three tilings
    python3 models/_tb.py --tests-only
"""
import math
import os
import sys

import numpy as np
import scipy.sparse as sp

_ROOT = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

FAMILY = {"hat": (1.0, math.sqrt(3.0)), "spectre": (1.0, 1.0), "turtle": (math.sqrt(3.0), 1.0)}


# ------------------------------------------------------------------------------------------------ graph

class Graph(object):
    """Points (n, d), bonds (m, 2) int with i < j (each bond once), and optionally the faces (lists of site indices).

    ``length`` are the straight distances between the bond ends; a model that has its own bond lengths (curved edges, say)
    can pass them to ``hopping_values`` instead.  ``data`` is free for the caller, as everywhere in these models.
    """

    def __init__(self, points, bonds, faces=None, data=None):
        self.points = np.asarray(points, float)
        self.bonds = np.asarray(bonds, int).reshape(-1, 2)
        self.faces = [list(f) for f in faces] if faces is not None else []
        self.data = dict(data or {})
        if len(self.bonds) and (self.bonds.min() < 0 or self.bonds.max() >= len(self.points)):
            raise ValueError("bond index out of range")
        if np.any(self.bonds[:, 0] == self.bonds[:, 1]):
            raise ValueError("a bond joins a site to itself")

    @property
    def n(self):
        return len(self.points)

    @property
    def length(self):
        d = self.points[self.bonds[:, 0]] - self.points[self.bonds[:, 1]]
        return np.sqrt((d * d).sum(axis=1))

    def degree(self):
        return np.bincount(self.bonds.ravel(), minlength=self.n)

    def euler(self):
        """V - E + F; equals 1 for a simply connected patch of faces."""
        return self.n - len(self.bonds) + len(self.faces)

    def is_bipartite(self):
        """Two-colouring by breadth-first search (so a spectrum symmetric about 0 is expected)."""
        A = adjacency(self.n, self.bonds, np.ones(len(self.bonds))).tocsr()
        col = -np.ones(self.n, int)
        for s in range(self.n):
            if col[s] >= 0:
                continue
            col[s] = 0
            stack = [s]
            while stack:
                u = stack.pop()
                for v in A.indices[A.indptr[u]:A.indptr[u + 1]]:
                    if col[v] < 0:
                        col[v] = 1 - col[u]
                        stack.append(v)
                    elif col[v] == col[u]:
                        return False
        return True


def patch_graph(levels, a=1.0, b=1.0, decimals=5):
    """The vertex graph of a level-n Spectre-family patch (a, b): one site per tiling vertex, one bond per tile edge.

    Vertices of different tiles are merged when their coordinates agree to ``decimals`` places (a choice; the tiles meet
    exactly, so any value between about 3 and 8 gives the same graph for levels up to 3).  The polygons are the straight
    ones of ``SpectrePatch`` (14 corners each), so a *tile edge* is one bond.
    """
    from spectre_patch import SpectrePatch
    P = SpectrePatch(levels, a, b)
    key, bonds, faces = {}, set(), []
    for i in range(P.n_tiles):
        ids = [key.setdefault((round(float(p[0]), decimals) + 0.0, round(float(p[1]), decimals) + 0.0), len(key))
               for p in P.straight_polygon(i)]
        faces.append(ids)
        for j in range(len(ids)):
            u, v = ids[j], ids[(j + 1) % len(ids)]
            if u != v:
                bonds.add((min(u, v), max(u, v)))
    pts = np.array(sorted(key, key=key.get))
    return Graph(pts, sorted(bonds), faces, {"levels": levels, "a": a, "b": b})


# ------------------------------------------------------------------------------------------------ Hamiltonian

def hopping_values(length, hopping="unit", t0=1.0, kappa=1.0):
    """'unit': t0;  'inverse': t0 / length;  'gauss': t0 exp(-kappa (length^2 - 1))."""
    length = np.asarray(length, float)
    if hopping == "unit":
        return t0 * np.ones_like(length)
    if hopping == "inverse":
        return t0 / length
    if hopping == "gauss":
        return t0 * np.exp(-kappa * (length ** 2 - 1.0))
    raise ValueError("unknown hopping %r" % (hopping,))


def adjacency(n, bonds, values):
    """Symmetric sparse (n, n) matrix with values[k] on the bond k (both directions)."""
    e = np.asarray(bonds, int).reshape(-1, 2)
    v = np.asarray(values)
    return sp.csr_matrix((np.r_[v, v], (np.r_[e[:, 0], e[:, 1]], np.r_[e[:, 1], e[:, 0]])), shape=(n, n))


def tight_binding(graph, hopping="unit", t0=1.0, kappa=1.0, onsite=None, values=None):
    """Sparse H.  ``values`` overrides the hopping rule (one number per bond, real or complex, for Hermitian use see
    ``adjacency``); ``onsite`` is one number per site."""
    if values is None:
        values = hopping_values(graph.length, hopping, t0, kappa)
    H = adjacency(graph.n, graph.bonds, values)
    if onsite is not None:
        onsite = np.asarray(onsite)
        if onsite.shape != (graph.n,):
            raise ValueError("onsite needs one number per site")
        H = H + sp.diags(onsite)
    return H.tocsr()


def spectrum(H):
    """(eigenvalues ascending, eigenvectors as columns); dense, so for patches up to a few thousand sites."""
    M = H.toarray() if sp.issparse(H) else np.asarray(H)
    return np.linalg.eigh(M)


def ipr(vectors):
    """Inverse participation ratio sum_i |psi_i|^4 of every column.  Between 1/N (spread evenly) and 1 (one site).
    N * IPR stays bounded for extended states and grows with N for localised ones."""
    return (np.abs(vectors) ** 4).sum(axis=0)


def summary(w, v):
    n = len(w)
    p = ipr(v)
    return dict(n=n, emin=float(w.min()), emax=float(w.max()), zero=int((abs(w) < 1e-8).sum()),
                sym=float(abs(w + w[::-1]).max()), nipr_mean=float(p.mean() * n), nipr_median=float(np.median(p) * n),
                ipr_max=float(p.max()))


def dos(w, bins=120, window=None):
    """Histogram density of states: (centres, density)."""
    h, e = np.histogram(w, bins=bins, range=window, density=True)
    return 0.5 * (e[1:] + e[:-1]), h


def to_pyqula(graph, H):
    """Optional: a finite spinless pyqula ``Hamiltonian`` with these positions and hopping matrix (needs pyqula)."""
    from pyqula import geometry
    pts = graph.points
    if pts.shape[1] == 2:
        pts = np.c_[pts, np.zeros(len(pts))]
    g = geometry.Geometry()
    g.dimensionality = 0
    g.r, g.x, g.y, g.z = pts, pts[:, 0], pts[:, 1], pts[:, 2]
    h = g.get_hamiltonian(has_spin=False)
    h.intra = sp.csc_matrix(H).astype(complex)
    return h


# ------------------------------------------------------------------------------------------------ figure

def figure_tilings(level=3, save=None, show=True):
    """Figure 1: density of states of Hat, Spectre, Turtle for unit and 1/length hopping.
    Figure 2: N * IPR of every eigenstate of the Spectre patch (unit hopping)."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    cols = dict(hat="tab:blue", spectre="tab:orange", turtle="tab:green")
    spec_v = None
    for k, (a, b) in FAMILY.items():
        g = patch_graph(level, a, b)
        for j, hop in enumerate(("unit", "inverse")):
            w, v = spectrum(tight_binding(g, hop))
            x, h = dos(w)
            ax[j].plot(x, h, color=cols[k], lw=1.4 if k != "spectre" else 2.4, ls="-" if k != "spectre" else "--", label=k)
            if k == "spectre" and hop == "unit":
                spec_v = (w, ipr(v) * len(w))
    ax[0].set_title("unit hopping: the three curves coincide")
    ax[1].set_title("hopping = 1 / bond length")
    for x in ax[:2]:
        x.set_xlabel("E")
        x.set_ylabel("density of states")
        x.legend()
    ax[2].scatter(*spec_v, s=2, color="tab:orange")
    ax[2].set_title("Spectre, unit hopping: N * IPR")
    ax[2].set_xlabel("E")
    fig.suptitle("finite level-%d patches (with boundary), %d sites" % (level, g.n))
    fig.tight_layout()
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=110)
        print("wrote", save)
    if show:
        plt.show()
    plt.close(fig)


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    s3 = math.sqrt(3.0)
    g = {k: patch_graph(2, *ab) for k, ab in FAMILY.items()}
    sp2 = g["spectre"]
    check("level-2 patch has V, E, F = 518, 588, 71 and V - E + F = 1",
          (sp2.n, len(sp2.bonds), len(sp2.faces), sp2.euler()) == (518, 588, 71, 1))
    g3 = patch_graph(3, 1, 1)
    check("level-3 patch has V, E, F = 3734, 4292, 559 (the numbers quoted by quantum-lattice)",
          (g3.n, len(g3.bonds), len(g3.faces), g3.euler()) == (3734, 4292, 559, 1))
    check("Hat, Spectre and Turtle have the same bonds and the same number of sites",
          all(np.array_equal(g[k].bonds, sp2.bonds) and g[k].n == sp2.n for k in g))
    check("Spectre bonds all have length 1; Hat and Turtle only 1 and sqrt3, and both occur",
          np.allclose(sp2.length, 1.0, atol=1e-4)
          and all(np.all(np.isclose(g[k].length, 1.0, atol=1e-4) | np.isclose(g[k].length, s3, atol=1e-4))
                  and np.isclose(g[k].length, s3, atol=1e-4).any() for k in ("hat", "turtle")))
    check("site degrees are 2, 3 or 4", set(sp2.degree()) <= {2, 3, 4}, "(%s)" % sorted(int(x) for x in set(sp2.degree())))
    check("the vertex graph is bipartite", sp2.is_bipartite())
    check("merge tolerance: 4 and 7 decimals give the same graph",
          all(len(patch_graph(2, 1, 1, d).bonds) == 588 and patch_graph(2, 1, 1, d).n == 518 for d in (4, 7)))

    res = {}
    for k in g:
        for hop in ("unit", "inverse"):
            H = tight_binding(g[k], hop)
            res[(k, hop)] = (H,) + spectrum(H)
    w_s = res[("spectre", "unit")][1]
    check("unit hopping: Hat and Turtle spectra equal the Spectre spectrum",
          all(abs(res[(k, "unit")][1] - w_s).max() < 1e-10 for k in ("hat", "turtle")))
    check("1/length hopping: the three spectra differ pairwise",
          all(abs(res[(p, "inverse")][1] - res[(q, "inverse")][1]).max() > 0.05
              for p, q in (("hat", "spectre"), ("turtle", "spectre"), ("hat", "turtle"))))
    check("every spectrum is symmetric about 0 (bipartite)", all(abs(r[1] + r[1][::-1]).max() < 1e-9 for r in res.values()))
    check("trace identity: sum E^2 = sum H_ij^2",
          all(abs((r[1] ** 2).sum() - r[0].multiply(r[0]).sum()) < 1e-8 for r in res.values()))
    check("eigenvectors are orthonormal and satisfy H v = E v",
          all(abs(r[2].T @ r[2] - np.eye(len(r[1]))).max() < 1e-9 and abs(r[0] @ r[2] - r[2] * r[1]).max() < 1e-9
              for r in res.values()))
    check("1/N <= IPR <= 1 for every state", all(((ipr(r[2]) >= 1.0 / len(r[1]) - 1e-12) & (ipr(r[2]) <= 1 + 1e-12)).all()
                                                    for r in res.values()))
    w3 = np.linalg.eigvalsh(tight_binding(g3).toarray())
    check("level 3, unit hopping: E in [-2.434, 2.434] (quantum-lattice TILINGS.md)",
          abs(w3.max() - 2.434) < 5e-4 and abs(w3.min() + 2.434) < 5e-4, "(%.4f)" % w3.max())

    # small exact cases, to see that the helpers do what they say
    ring = Graph(np.c_[np.cos(2 * np.pi * np.arange(6) / 6), np.sin(2 * np.pi * np.arange(6) / 6)],
                 [(i, (i + 1) % 6) if i < 5 else (0, 5) for i in range(6)])
    wr = spectrum(tight_binding(ring))[0]
    check("6-ring: eigenvalues 2 cos(2 pi k / 6)", abs(np.sort(wr) - np.sort(2 * np.cos(2 * np.pi * np.arange(6) / 6))).max() < 1e-12)
    check("6-ring is bipartite, a 5-ring is not",
          ring.is_bipartite() and not Graph(np.zeros((5, 2)), [(i, (i + 1) % 5) if i < 4 else (0, 4) for i in range(5)]).is_bipartite())
    on = tight_binding(ring, onsite=np.arange(6.0))
    check("onsite energies sit on the diagonal", abs(on.diagonal() - np.arange(6.0)).max() == 0)
    check("hopping rules: unit, inverse, gauss",
          np.allclose(hopping_values([1, 2], "unit", 3.0), 3.0) and np.allclose(hopping_values([1, 2], "inverse"), [1, 0.5])
          and np.allclose(hopping_values([1, 2], "gauss", 1.0, 0.5), [1, math.exp(-1.5)]))
    flat = tight_binding(Graph(np.zeros((3, 1)), [(0, 1), (1, 2)]), values=[1.0, 3.0])
    check("explicit bond values override the rule", flat[1, 2] == 3.0 and flat[2, 1] == 3.0)
    one = np.zeros((4, 1))
    one[0, 0] = 1
    check("IPR of a site state is 1 and of an even state is 1/N", abs(ipr(one)[0] - 1) < 1e-15 and abs(ipr(np.ones((4, 1)) / 2)[0] - 0.25) < 1e-15)
    bad = False
    try:
        Graph(np.zeros((2, 2)), [(0, 2)])
    except ValueError:
        bad = True
    check("a bond to a missing site is refused", bad)
    # optional: pyqula round trip when it is installed
    try:
        import pyqula  # noqa: F401
    except Exception:
        check("pyqula export (skipped: pyqula not installed)", True)
    else:
        h = to_pyqula(sp2, res[("spectre", "unit")][0])
        check("pyqula Hamiltonian holds the same matrix", abs(np.asarray(h.intra.todense()) - res[("spectre", "unit")][0].toarray()).max() < 1e-12)
    return ok


def main(argv):
    if "--skip-tests" not in argv:
        print("tests:")
        if not run_tests():
            print("TESTS FAILED")
            return 1
        if "--tests-only" in argv:
            return 0
    elif "--tests-only" in argv:
        return 0
    print("level-3 patches (finite, with boundary)")
    for k, (a, b) in FAMILY.items():
        g = patch_graph(3, a, b)
        for hop in ("unit", "inverse"):
            w, v = spectrum(tight_binding(g, hop))
            s = summary(w, v)
            print("  %-8s %-8s N=%d  E in [%.3f, %.3f]  zero modes %d  mean N*IPR %.1f  max IPR %.3f"
                  % (k, hop, s["n"], s["emin"], s["emax"], s["zero"], s["nipr_mean"], s["ipr_max"]))
    save = None
    if "--save-dir" in argv:
        save = os.path.join(argv[argv.index("--save-dir") + 1], "tb_tilings.png")
    figure_tilings(3, save=save, show="--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
