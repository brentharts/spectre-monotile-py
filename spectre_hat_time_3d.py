"""
spectre_hat_time_3d.py - the figure of spectre_hat_time_model.py in three dimensions: the expansion and collapse of the 14th vertex.

The 2D figure (figures/spectre_hat_time.png) shows five slices of the flow and the curves d(tau), omega(tau).  Here the flow parameter tau is the vertical axis, so the whole Spectre -> Hat
flow is one solid and the bit at P_8 is something you can walk around.

    Left panel, 'loft':  every outline of the flow stacked along tau (x, y = the tile, z = tau).  The smooth sector (every edge except 7 and 8) is the grey sheet; edges 7 and 8 of the two bit
                         values are the blue (s = +1) and red (s = -1) sheets, which part at the Spectre end and close up at the Hat.  Black: the worldline of P_8 that the record of the run
                         actually follows (it jumps between the two branches at a snap).  Rings: the outline at tau = 0, 1/4, 1/2, 3/4, 1.
    Right panel, 'horn': the neighbourhood of the 14th vertex only, in the frame of the flat vertex (x = displacement of P_8 along the chord normal, y = along the chord).  The two branches
                         are P_8 = +-h(tau), h = a tan(omega / 2); the band between them is the jump d(tau) = 2h, coloured by readability d(tau)/d(0).  It is a horn that is widest at the
                         Spectre (expansion of the bit's two values) and pinches to a point at the Hat (collapse: the two tiles are the same Hat).  Orange plane: the start tau* of the
                         window in which snaps are allowed (rule R2); below it the bit is set in stone.

Same parameters as the 2D figure (omega_0 = 120 degrees, snap scale 0.5, 20 ticks) and the same scripted story, so the snaps (triangle: rewrite of an earlier tick, circle: flip of the
present) are the ones of the 2D figure.

    python3 spectre_hat_time_3d.py --no-show --save figures/spectre_hat_time_3d.png
    python3 spectre_hat_time_3d.py --no-show --gif figures/spectre_hat_time_3d.gif      # a turntable
    python3 spectre_hat_time_3d.py --tests-only

Needs numpy and matplotlib (and Pillow for --gif); the model itself needs only numpy.
"""

import argparse
import math
import sys

import numpy as np

from spectre_hat_collinearity_flow_model import FLAT_VERTEX
from spectre_hat_time_model import SpectreHatTime, is_simple_tile, story

__all__ = ["run_story", "loft_strip", "horn_coordinates", "worldline", "plot3d", "run_tests"]

OPENED = (FLAT_VERTEX - 1, FLAT_VERTEX)  # edges 7 and 8
SMOOTH_ORDER = [e for e in list(range(FLAT_VERTEX + 1, 14)) + list(range(0, FLAT_VERTEX - 1))]  # 9..13, 0..6: one contiguous strip around the other side of P_8
BLUE, RED, GREY, ORANGE = "tab:blue", "tab:red", "0.55", "tab:orange"


def run_story(ticks=20, corner=120.0, snap_scale=0.5, k0=0.5):
    """The model after the scripted story of spectre_hat_time_model.story (omega_0 = 120 degrees, snap scale 0.5: the 2D figure)."""
    m = SpectreHatTime(ticks=ticks, k0=k0, corner_deg=corner, snap_scale=snap_scale)
    story(m, out=lambda *a, **k: None)
    return m


# --------------------------------------------------------------------------
# geometry for the plots
# --------------------------------------------------------------------------
def loft_strip(m, taus, s, edges, samples=24):
    """Stack the sampled edges ``edges`` (in that order) of the tile with bit s over ``taus``: arrays X, Y, Z of shape (len(taus), len(edges) * samples)."""
    X, Y = [], []
    for tv in taus:
        pts = np.asarray(m.tile(tv, s).sample_edges(samples - 1))  # (14, samples, 2)
        row = np.concatenate([pts[e] for e in edges], axis=0)
        X.append(row[:, 0])
        Y.append(row[:, 1])
    X, Y = np.array(X), np.array(Y)
    return X, Y, np.repeat(np.asarray(taus, dtype=float)[:, None], X.shape[1], axis=1)


def flat_position(m, tau):
    """Where P_8 sits when it is flat (on the chord P_7 -> P_9) at tau: the origin of the horn frame."""
    p = m.tile(tau, 1).vertices[FLAT_VERTEX].position
    return p - np.array([m.flow.opening_of(tau), 0.0])


def horn_coordinates(m, taus, s, samples=24):
    """Edges 7 and 8 of the tile with bit s in the frame of the flat vertex, over ``taus``: X, Y, Z of shape (len(taus), 2 * samples)."""
    X, Y, Z = loft_strip(m, taus, s, OPENED, samples)
    o = np.array([flat_position(m, tv) for tv in taus])
    return X - o[:, 0:1], Y - o[:, 1:2], Z


def worldline(m, n_per_tick=6):
    """The P_8 worldline the record follows: list of (tau, x_rel, bit) with x_rel = bit * h(tau) in the horn frame; the bit changes only at ticks, which gives the jumps."""
    out = []
    for j in range(m.N):
        s = m.record[j] if j <= m.now else m.bit
        for tv in np.linspace(m.tau_of(j), m.tau_of(j + 1), n_per_tick, endpoint=False):
            out.append((float(tv), s * m.flow.opening_of(tv), s))
    out.append((1.0, 0.0, m.record[-1]))
    return out


def _snap_points(m, frame):
    """(tau, x, kind) of the snaps of the log, on the branch the snap moved to (horn frame) or at P_8 (loft frame)."""
    pts = []
    for e in m.log:
        tau = e["tau"]
        s_new = e["new"]
        if frame == "horn":
            pts.append((tau, s_new * m.flow.opening_of(tau), e["kind"]))
        else:
            p = m.tile(tau, s_new).vertices[FLAT_VERTEX].position
            pts.append((tau, p, e["kind"]))
    return pts


# --------------------------------------------------------------------------
# plotting
# --------------------------------------------------------------------------
def plot3d(m, fig=None, n_tau=61, azim=(-58.0, -72.0), elev=(22.0, 13.0)):
    """Draw the two panels on ``fig`` (created if None).  Returns (fig, ax_loft, ax_horn)."""
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from matplotlib.colors import Normalize

    fig = fig or plt.figure(figsize=(15.0, 7.4))
    taus = np.linspace(0.0, 1.0, n_tau)
    ts = m.tau_star()

    # ---- left: the loft -------------------------------------------------------------------------------------------
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    X, Y, Z = loft_strip(m, taus, 1, SMOOTH_ORDER)
    ax.plot_surface(X, Y, Z, color=GREY, alpha=0.16, linewidth=0, antialiased=True, shade=True)
    for s, col in ((1, BLUE), (-1, RED)):
        X, Y, Z = loft_strip(m, taus, s, OPENED)
        ax.plot_surface(X, Y, Z, color=col, alpha=0.45, linewidth=0, antialiased=True, shade=True)
    for tv in (0.0, 0.25, 0.5, 0.75, 1.0):
        for s, col, edges in ((1, GREY, SMOOTH_ORDER), (1, BLUE, OPENED), (-1, RED, OPENED)):
            x, y, z = loft_strip(m, [tv], s, edges)
            ax.plot(x[0], y[0], z[0], color=col, lw=1.1 if col == GREY else 1.6)
    # the outline at the start of the snap window
    for s, col, edges in ((1, ORANGE, SMOOTH_ORDER + list(OPENED)),):
        x, y, z = loft_strip(m, [ts], s, edges)
        ax.plot(x[0], y[0], z[0], color=col, lw=2.0)
    p_plus = np.array([m.tile(tv, 1).vertices[FLAT_VERTEX].position for tv in taus])
    p_minus = np.array([m.tile(tv, -1).vertices[FLAT_VERTEX].position for tv in taus])
    ax.plot(p_plus[:, 0], p_plus[:, 1], taus, color=BLUE, lw=2.2)
    ax.plot(p_minus[:, 0], p_minus[:, 1], taus, color=RED, lw=2.2)
    wl = worldline(m)
    wp = np.array([m.tile(t, s).vertices[FLAT_VERTEX].position for t, _, s in wl])
    ax.plot(wp[:, 0], wp[:, 1], [t for t, _, _ in wl], color="k", lw=2.6, zorder=10)
    for tau, p, kind in _snap_points(m, "loft"):
        ax.scatter([p[0]], [p[1]], [tau], marker="v" if kind == "snap_back" else "o", color="k", s=60, depthshade=False, zorder=11)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("tau  (Spectre -> Hat)")
    ax.set_title("the flow as a solid: tile outlines stacked along tau\n(blue/red: the two values of the bit at the 14th vertex; black: the worldline of the record)", fontsize=9)
    xs, ys = X.ravel(), Y.ravel()
    _equal_xy(ax, np.array([p_plus[:, 0], p_minus[:, 0]]).ravel(), np.array([p_plus[:, 1], p_minus[:, 1]]).ravel(), X, Y, SMOOTH_ORDER, m, taus)
    ax.set_zlim(0.0, 1.0)
    ax.view_init(elev=elev[0], azim=azim[0])

    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], color=BLUE, lw=3, label="bit s = +1 (convex corner at P_8)"),
               Line2D([0], [0], color=RED, lw=3, label="bit s = -1 (reflex corner)"),
               Line2D([0], [0], color="k", lw=3, label="worldline of the record"),
               Line2D([0], [0], color="k", lw=0, marker="v", ms=8, label="rewrite of an earlier tick"),
               Line2D([0], [0], color="k", lw=0, marker="o", ms=7, label="flip of the present"),
               Line2D([0], [0], color=ORANGE, lw=3, label="tau* (start of the snap window)")]
    fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=8, frameon=False)

    # ---- right: the horn -------------------------------------------------------------------------------------------
    bx = fig.add_subplot(1, 2, 2, projection="3d")
    h = np.array([m.flow.opening_of(tv) for tv in taus])
    readable = np.array([m.readable(tv) for tv in taus])
    # the band between the two branches: x from -h to +h at y = 0, coloured by readability
    Xb = np.stack([-h, h], axis=1)
    Yb = np.zeros_like(Xb)
    Zb = np.stack([taus, taus], axis=1)
    norm = Normalize(0.0, 1.0)
    fc = cm.viridis(norm(0.5 * (readable[:-1] + readable[1:])))[:, None, :]
    bx.plot_surface(Xb, Yb, Zb, facecolors=fc, shade=False, alpha=0.9, linewidth=0, antialiased=True)
    for s, col in ((1, BLUE), (-1, RED)):
        X, Y, Z = horn_coordinates(m, taus, s)
        bx.plot_surface(X, Y, Z, color=col, alpha=0.16, linewidth=0, antialiased=True, shade=True)
        for tv in (0.0, 0.25, 0.5, 0.75, 1.0):
            x, y, z = horn_coordinates(m, [tv], s)
            bx.plot(x[0], y[0], z[0], color=col, lw=1.2)
        bx.plot(s * h, np.zeros_like(h), taus, color=col, lw=2.4)
    span = 1.15 * float(h.max())
    bad = [float(tv) for tv in taus if not is_simple_tile(m.tile(tv, -1))]
    if bad:  # the reflex outline is not a simple closed curve there (E0.4 of research/toymodel_v4): the bit is forced to +1
        lo, hi = min(bad), max(bad)
        sel = (taus >= lo - 1e-12) & (taus <= hi + 1e-12)
        bx.plot(-h[sel], np.zeros(sel.sum()), taus[sel], color="k", lw=3.2, ls=":", zorder=9)
        bx.text(-span, -1.1, hi + 0.06, "s = -1 forbidden for tau <= %.2f:\nits outline self-intersects" % hi, color="k", fontsize=8, ha="left")
        xx0, yy0 = np.meshgrid([-span, span], [-1.05, 1.05])
        for tz in (lo, hi):
            bx.plot_surface(xx0, yy0, np.full_like(xx0, tz), color="0.3", alpha=0.10, linewidth=0, shade=False)
    bx.plot([0, 0], [0, 0], [0, 1], color="0.35", lw=1.0, ls=":")  # the flat vertex
    wl = worldline(m)
    bx.plot([x for _, x, _ in wl], np.zeros(len(wl)), [t for t, _, _ in wl], color="k", lw=2.6, zorder=10)
    for tau, x, kind in _snap_points(m, "horn"):
        bx.scatter([x], [0.0], [tau], marker="v" if kind == "snap_back" else "o", color="k", s=60, depthshade=False, zorder=11)
    span = 1.15 * float(h.max())
    xx, yy = np.meshgrid([-span, span], [-1.05, 1.05])
    bx.plot_surface(xx, yy, np.full_like(xx, ts), color=ORANGE, alpha=0.18, linewidth=0, shade=False)
    bx.text(-span, -1.1, ts + 0.035, "tau* = %.2f: snap window above" % ts, color="tab:orange", fontsize=8, ha="left")
    bx.text(-span, -1.1, ts - 0.05, "set in stone below", color="0.4", fontsize=8, ha="left")
    bx.text(span, 1.1, 0.02, "expansion: d = %.2f" % m.jump(0.0), color="k", fontsize=8, ha="right")
    bx.text(0.0, 0.0, 1.05, "collapse: d = 0 (the Hat)", color="k", fontsize=8, ha="center")
    h0 = round(float(h.max()), 1)
    bx.set_xticks([-h0, 0.0, h0])
    bx.set_yticks([-1.0, 0.0, 1.0])
    bx.set_xlabel("P_8 displacement  (+-h)")
    bx.set_ylabel("along the chord")
    bx.set_zlabel("tau")
    bx.set_xlim(-span, span)
    bx.set_ylim(-1.1, 1.1)
    bx.set_zlim(0.0, 1.0)
    bx.set_box_aspect((1.0, 0.7, 1.2))
    bx.set_title("the 14th vertex: expansion and collapse of the bit\n(band = jump d(tau) = 2h, colour = readability; it pinches shut at the Hat)", fontsize=9)
    bx.view_init(elev=elev[1], azim=azim[1])
    sm = cm.ScalarMappable(norm=norm, cmap="viridis")
    sm.set_array([])
    cb = fig.colorbar(sm, ax=bx, shrink=0.5, pad=0.08)
    cb.set_label("readable  d(tau)/d(0)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    fig.suptitle("Spectre -> Hat flow with a bit at P_8   (omega_0 = %g deg, snap scale %g, %d ticks)" % (m.flow.corner_deg, m.snap_scale, m.N), fontsize=11)
    fig.subplots_adjust(left=0.01, right=0.97, bottom=0.07, top=0.88, wspace=0.02)
    return fig, ax, bx


def _equal_xy(ax, px, py, X, Y, order, m, taus):
    """Equal scale in x and y for the loft panel (the tile is not stretched)."""
    allx = np.concatenate([np.asarray(X).ravel(), np.asarray(px).ravel()])
    ally = np.concatenate([np.asarray(Y).ravel(), np.asarray(py).ravel()])
    for tv in (0.0, 1.0):
        pts = np.asarray(m.tile(tv, 1).sample_edges(20)).reshape(-1, 2)
        allx = np.concatenate([allx, pts[:, 0]])
        ally = np.concatenate([ally, pts[:, 1]])
    cx, cy = 0.5 * (allx.min() + allx.max()), 0.5 * (ally.min() + ally.max())
    r = 0.5 * max(allx.max() - allx.min(), ally.max() - ally.min())
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_box_aspect((1.0, 1.0, 1.25))


def turntable(m, path, frames=48, dpi=60):
    """Write a rotating view of both panels to ``path`` (a GIF; needs Pillow)."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    fig = plt.figure(figsize=(13.0, 6.4))
    fig, ax, bx = plot3d(m, fig, n_tau=41)

    def update(i):
        f = 360.0 * i / frames
        ax.view_init(elev=20.0 + 6.0 * math.sin(math.radians(f)), azim=-58.0 + f)
        bx.view_init(elev=18.0 + 6.0 * math.sin(math.radians(f)), azim=-35.0 + f)
        return ()

    anim = FuncAnimation(fig, update, frames=frames, interval=70, blit=False)
    anim.save(path, writer=PillowWriter(fps=12), dpi=dpi)
    plt.close(fig)


# --------------------------------------------------------------------------
# self tests (geometry of the plots, not of the pictures)
# --------------------------------------------------------------------------
def test_horn_branches_are_plus_minus_h_and_merge_at_the_hat():
    m = run_story()
    for tv in (0.0, 0.3, 0.7, 1.0):
        h = m.flow.opening_of(tv)
        for s in (1, -1):
            x, y, z = horn_coordinates(m, [tv], s)
            # P_8 is the last sample of edge 7 (index samples - 1): its horn coordinates are (s h, 0)
            assert abs(x[0][23] - s * h) < 1e-9 and abs(y[0][23]) < 1e-9, (tv, s, x[0][23], h)
    x1, _, _ = horn_coordinates(m, [1.0], 1)
    x2, _, _ = horn_coordinates(m, [1.0], -1)
    assert np.allclose(x1, x2, atol=1e-12), "the two branches are the same curve at the Hat"


def test_loft_rows_are_the_sampled_outline_and_z_is_tau():
    m = run_story()
    taus = np.linspace(0.0, 1.0, 5)
    X, Y, Z = loft_strip(m, taus, -1, list(range(14)), samples=10)
    assert X.shape == (5, 140) and np.all(Z[:, 0] == taus) and np.all(np.diff(Z[:, 0]) > 0)
    pts = np.asarray(m.tile(0.5, -1).sample_edges(9)).reshape(-1, 2)
    assert np.allclose(np.column_stack([X[2], Y[2]]), pts)
    # the smooth sector is the same for both bits (rule R4 of the model), so the grey sheet can be drawn once
    Xp, Yp, _ = loft_strip(m, taus, 1, SMOOTH_ORDER)
    Xm, Ym, _ = loft_strip(m, taus, -1, SMOOTH_ORDER)
    assert np.allclose(Xp, Xm) and np.allclose(Yp, Ym)


def test_worldline_follows_the_record_and_jumps_by_d_at_ticks():
    m = run_story()
    wl = worldline(m)
    taus = [t for t, _, _ in wl]
    assert all(b >= a for a, b in zip(taus, taus[1:]))
    for (t, x, s) in wl:
        assert abs(x - s * m.flow.opening_of(t)) < 1e-12
    for j in range(1, m.N):
        before = [w for w in wl if w[0] < m.tau_of(j) - 1e-12][-1]
        after = [w for w in wl if abs(w[0] - m.tau_of(j)) < 1e-12][0]
        if m.record[j - 1] != m.record[j]:
            jump = abs(after[1] - m.flow.opening_of(m.tau_of(j)) * m.record[j - 1])
            assert abs(jump - m.jump(m.tau_of(j))) < 1e-9, j
        else:
            assert after[2] == before[2]
    assert [e["kind"] for e in m.log] == ["snap_back", "flip_now", "flip_now"], "the story of the 2D figure"


def run_tests(verbose=True):
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-80s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def main(argv=None):
    p = argparse.ArgumentParser(description="The Spectre -> Hat flow with a bit at the 14th vertex, in 3D (tau vertical).")
    p.add_argument("--corner", type=float, default=120.0, metavar="DEG", help="omega_0 in degrees (default 120: the 2D figure)")
    p.add_argument("--snap-scale", type=float, default=0.5, help="largest jump a snap may cause (default 0.5: the 2D figure)")
    p.add_argument("--ticks", type=int, default=20, help="number of ticks (default 20)")
    p.add_argument("--k0", type=float, default=0.5, help="curve strength at tau = 0 (default 0.5)")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit")
    p.add_argument("--no-show", action="store_true", help="do not open a window")
    p.add_argument("--save", metavar="PNG", help="save the figure to this file")
    p.add_argument("--gif", metavar="GIF", help="save a turntable animation to this file (needs Pillow)")
    args = p.parse_args(argv)
    if args.tests_only:
        print("spectre_hat_time_3d self tests:")
        return 0 if run_tests() else 1
    m = run_story(args.ticks, args.corner, args.snap_scale, args.k0)
    import matplotlib

    if args.no_show or args.save or args.gif:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if args.save or not args.gif:
        fig, _, _ = plot3d(m)
        if args.save:
            fig.savefig(args.save, dpi=130)
            print("saved", args.save)
        if not args.no_show and not args.save:
            plt.show()
        plt.close(fig)
    if args.gif:
        turntable(m, args.gif)
        print("saved", args.gif)
    return 0


if __name__ == "__main__":
    sys.exit(main())
