// spectre_fast_cavity.cpp - Ultra-fast exact (1/2) Z[sqrt(3)] C++20 engine for:
//   Mode "mrv": Unrestricted Combinatorial MRV-Edge Cavity Solver (Engine A, zero hull assumption!)
//   Mode "cnf": Exact Spatial-Bucketed Overlap + Stokes-Flux CNF Generator (Engine B, feeds CaDiCaL + drat-trim!)
//
// Zero floating-point tolerances in any geometric predicate (128-bit exact Z[sqrt(3)] arithmetic!).
// Compiled with: g++ -O3 -march=native -std=c++20 -fopenmp spectre_fast_cavity.cpp -o bin/spectre_fast_cavity
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

static constexpr double SQRT3 = 1.7320508075688772935;

using i64 = int64_t;
using i128 = __int128_t;

struct Z3Num {
    i64 a, b; // represents (a + b*sqrt(3)) / 2
    bool operator==(const Z3Num& o) const { return a == o.a && b == o.b; }
    bool operator!=(const Z3Num& o) const { return !(*this == o); }
};

struct Z3Pt {
    Z3Num x, y;
    bool operator==(const Z3Pt& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Z3Pt& o) const { return !(*this == o); }
};

struct Z4Vec {
    i64 v[4];
    bool operator==(const Z4Vec& o) const {
        return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2] && v[3] == o.v[3];
    }
    bool operator<(const Z4Vec& o) const {
        for (int i = 0; i < 4; ++i) if (v[i] != o.v[i]) return v[i] < o.v[i];
        return false;
    }
};

struct Z4Hash {
    size_t operator()(const Z4Vec& x) const noexcept {
        uint64_t h = 0x9e3779b97f4a7c15ULL;
        for (int i = 0; i < 4; ++i) {
            h ^= uint64_t(x.v[i]) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        }
        return size_t(h);
    }
};

struct DirEdge {
    Z4Vec u, v;
    bool operator==(const DirEdge& o) const { return u == o.u && v == o.v; }
    bool operator<(const DirEdge& o) const { return (u == o.u) ? (v < o.v) : (u < o.u); }
};

struct DirEdgeHash {
    size_t operator()(const DirEdge& e) const noexcept {
        Z4Hash zh;
        size_t h1 = zh(e.u), h2 = zh(e.v);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

struct PlacementKey {
    int orient;
    Z4Vec shift;
    bool operator==(const PlacementKey& o) const { return orient == o.orient && shift == o.shift; }
    bool operator<(const PlacementKey& o) const {
        return (orient != o.orient) ? (orient < o.orient) : (shift < o.shift);
    }
};

struct PlacementKeyHash {
    size_t operator()(const PlacementKey& k) const noexcept {
        size_t h = Z4Hash{}(k.shift);
        return h ^ (size_t(k.orient) * 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
    }
};

inline int z3_sign(i64 a, i64 b) {
    if (a == 0 && b == 0) return 0;
    if (a >= 0 && b >= 0) return 1;
    if (a <= 0 && b <= 0) return -1;
    i128 aa = i128(a) * i128(a);
    i128 bb3 = i128(3) * i128(b) * i128(b);
    if (a > 0) return (aa > bb3) ? 1 : ((aa < bb3) ? -1 : 0);
    return (bb3 > aa) ? 1 : ((bb3 < aa) ? -1 : 0);
}

inline int z3_cmp(Z3Num u, Z3Num v) { return z3_sign(u.a - v.a, u.b - v.b); }

inline int z3_orient(const Z3Pt& o, const Z3Pt& a, const Z3Pt& b) {
    i128 ax0 = a.x.a - o.x.a, ax1 = a.x.b - o.x.b;
    i128 ay0 = a.y.a - o.y.a, ay1 = a.y.b - o.y.b;
    i128 bx0 = b.x.a - o.x.a, bx1 = b.x.b - o.x.b;
    i128 by0 = b.y.a - o.y.a, by1 = b.y.b - o.y.b;
    i128 c0 = (ax0 * by0 + 3 * ax1 * by1) - (ay0 * bx0 + 3 * ay1 * bx1);
    i128 c1 = (ax0 * by1 + ax1 * by0) - (ay0 * bx1 + ay1 * bx0);
    if (c0 == 0 && c1 == 0) return 0;
    if (c0 >= 0 && c1 >= 0) return 1;
    if (c0 <= 0 && c1 <= 0) return -1;
    // Compare |c0|^2 vs 3*|c1|^2 using double? No, c0, c1 are <= 10^12 so c0*c0 fits in i128?
    // Coordinates are <= 10^4 -> diffs <= 10^4 -> c0, c1 <= 10^9 -> c0*c0 <= 10^18 fits easily in i128!
    i128 lhs = c0 * c0;
    i128 rhs = i128(3) * c1 * c1;
    if (c0 > 0) return (lhs > rhs) ? 1 : ((lhs < rhs) ? -1 : 0);
    return (rhs > lhs) ? 1 : ((rhs < lhs) ? -1 : 0);
}

inline bool z3_on_segment_strict(const Z3Pt& p, const Z3Pt& a, const Z3Pt& b) {
    if (p == a || p == b) return false;
    if (z3_orient(a, b, p) != 0) return false;
    return (z3_cmp(p.x, a.x) * z3_cmp(p.x, b.x) <= 0) &&
           (z3_cmp(p.y, a.y) * z3_cmp(p.y, b.y) <= 0);
}

inline bool z3_segments_properly_cross(const Z3Pt& a, const Z3Pt& b, const Z3Pt& c, const Z3Pt& d) {
    int o1 = z3_orient(a, b, c), o2 = z3_orient(a, b, d);
    if (o1 == 0 || o2 == 0 || o1 == o2) return false;
    int o3 = z3_orient(c, d, a), o4 = z3_orient(c, d, b);
    return o3 != 0 && o4 != 0 && o3 != o4;
}

inline Z3Pt z4_to_z3pt(const Z4Vec& v) {
    return {{2 * v.v[0] + v.v[1], v.v[2]}, {v.v[2] + 2 * v.v[3], v.v[1]}};
}

inline std::pair<double, double> z4_to_f2(const Z4Vec& v) {
    return {v.v[0] + 0.5 * v.v[1] + 0.5 * SQRT3 * v.v[2],
            0.5 * SQRT3 * v.v[1] + 0.5 * v.v[2] + v.v[3]};
}

static int z3_point_in_poly(const Z3Pt& p, const std::vector<Z3Pt>& poly) {
    int n = int(poly.size());
    bool inside = false;
    for (int i = 0; i < n; ++i) {
        const Z3Pt& a = poly[i];
        const Z3Pt& b = poly[(i + 1) % n];
        if (p == a || z3_on_segment_strict(p, a, b)) return 0;
        int ca = z3_cmp(a.y, p.y);
        int cb = z3_cmp(b.y, p.y);
        if ((ca > 0) != (cb > 0)) {
            if (cb > 0) {
                if (z3_orient(a, b, p) > 0) inside = !inside;
            } else {
                if (z3_orient(b, a, p) > 0) inside = !inside;
            }
        }
    }
    return inside ? 1 : -1;
}

struct TileGeom {
    PlacementKey key;
    std::array<Z4Vec, 14> verts;
    std::array<DirEdge, 14> edges;
    std::vector<Z3Pt> poly;
    std::vector<Z3Pt> poly2;
    double x0, y0, x1, y1;
};

static std::array<std::array<Z4Vec, 14>, 24> ORIENT_VERTS;
static std::array<std::array<int, 14>, 24> ORIENT_EDIRS;
static std::unordered_map<Z4Vec, int, Z4Hash> UNIT_DIR_MAP;
static std::vector<std::pair<int, int>> EDGES_BY_DIR[2][12];

static void init_spectre_tables() {
    // MZ: 1->z, w->zw, z->w, zw->w-1
    int MZ[4][4] = {{0,0,0,-1},{0,0,1,1},{1,0,0,0},{0,1,0,0}};
    int CONJ[4][4] = {{1,1,0,0},{0,-1,0,0},{0,0,1,0},{0,0,-1,-1}};
    int R[4][4];
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) R[i][j] = -CONJ[i][j];

    int ZP[12][4][4];
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) ZP[0][i][j] = (i == j);
    for (int k=1;k<12;++k) {
        for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
            int s = 0;
            for (int m=0;m<4;++m) s += MZ[i][m] * ZP[k-1][m][j];
            ZP[k][i][j] = s;
        }
    }
    for (int k=0;k<12;++k) {
        Z4Vec d{{ZP[k][0][0], ZP[k][1][0], ZP[k][2][0], ZP[k][3][0]}};
        UNIT_DIR_MAP[d] = k;
    }
    int edge_dirs[14] = {0, 10, 1, 3, 0, 2, 5, 7, 4, 6, 6, 8, 11, 9};
    std::array<Z4Vec, 14> base;
    base[0] = {{0, 0, 0, 0}};
    for (int i=0;i<13;++i) {
        int d = edge_dirs[i];
        for (int c=0;c<4;++c) base[i+1].v[c] = base[i].v[c] + ZP[d][c][0];
    }
    for (int ref=0; ref<2; ++ref) {
        for (int k=0; k<12; ++k) {
            int o = ref * 12 + k;
            int M[4][4];
            if (ref == 0) {
                for (int i=0;i<4;++i) for (int j=0;j<4;++j) M[i][j] = ZP[k][i][j];
            } else {
                for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
                    int s = 0;
                    for (int m=0;m<4;++m) s += ZP[k][i][m] * R[m][j];
                    M[i][j] = s;
                }
            }
            for (int i=0;i<14;++i) {
                Z4Vec p{};
                for (int r=0;r<4;++r) {
                    i64 s = 0;
                    for (int c=0;c<4;++c) s += M[r][c] * base[i].v[c];
                    p.v[r] = s;
                }
                ORIENT_VERTS[o][ref == 0 ? i : (13 - i)] = p;
            }
            for (int j=0;j<14;++j) {
                Z4Vec dv{};
                for (int c=0;c<4;++c) dv.v[c] = ORIENT_VERTS[o][(j+1)%14].v[c] - ORIENT_VERTS[o][j].v[c];
                int d = UNIT_DIR_MAP.at(dv);
                ORIENT_EDIRS[o][j] = d;
                if (ref == 0) EDGES_BY_DIR[0][d].push_back({o, j});
                EDGES_BY_DIR[1][d].push_back({o, j});
            }
        }
    }
}

static TileGeom make_tile(int orient, const Z4Vec& shift) {
    TileGeom tg;
    tg.key = {orient, shift};
    tg.poly.resize(14);
    tg.poly2.resize(14);
    tg.x0 = tg.y0 = 1e18;
    tg.x1 = tg.y1 = -1e18;
    for (int i=0;i<14;++i) {
        for (int c=0;c<4;++c) tg.verts[i].v[c] = ORIENT_VERTS[orient][i].v[c] + shift.v[c];
        tg.poly[i] = z4_to_z3pt(tg.verts[i]);
        tg.poly2[i] = {{2 * tg.poly[i].x.a, 2 * tg.poly[i].x.b}, {2 * tg.poly[i].y.a, 2 * tg.poly[i].y.b}};
        auto [fx, fy] = z4_to_f2(tg.verts[i]);
        tg.x0 = std::min(tg.x0, fx); tg.y0 = std::min(tg.y0, fy);
        tg.x1 = std::max(tg.x1, fx); tg.y1 = std::max(tg.y1, fy);
    }
    for (int i=0;i<14;++i) tg.edges[i] = {tg.verts[i], tg.verts[(i+1)%14]};
    return tg;
}

static bool has_pt(const std::vector<Z3Pt>& poly, const Z3Pt& p) {
    for (const auto& q : poly) if (q == p) return true;
    return false;
}

static bool tiles_overlap_exact(const TileGeom& A, const TileGeom& B) {
    if (A.x1 < B.x0 - 1e-6 || B.x1 < A.x0 - 1e-6 || A.y1 < B.y0 - 1e-6 || B.y1 < A.y0 - 1e-6) return false;
    // Check identical vertex sets
    bool all_same = true;
    for (int i=0;i<14;++i) if (!has_pt(B.poly, A.poly[i])) { all_same = false; break; }
    if (all_same) return true;
    // 1. Proper edge crossings
    for (int i=0;i<14;++i) {
        const Z3Pt& a1 = A.poly[i]; const Z3Pt& a2 = A.poly[(i+1)%14];
        for (int j=0;j<14;++j) {
            if (z3_segments_properly_cross(a1, a2, B.poly[j], B.poly[(j+1)%14])) return true;
        }
    }
    // 2. Vertices strictly inside
    for (int i=0;i<14;++i) if (!has_pt(B.poly, A.poly[i]) && z3_point_in_poly(A.poly[i], B.poly) == 1) return true;
    for (int j=0;j<14;++j) if (!has_pt(A.poly, B.poly[j]) && z3_point_in_poly(B.poly[j], A.poly) == 1) return true;
    // 3. Edge subsegment midpoints strictly inside
    std::vector<Z3Pt> splits;
    splits.reserve(6);
    auto check_edges = [&](const TileGeom& P, const TileGeom& Q) -> bool {
        for (int i=0;i<14;++i) {
            const Z3Pt& a1 = P.poly[i]; const Z3Pt& a2 = P.poly[(i+1)%14];
            splits.clear();
            splits.push_back(a1); splits.push_back(a2);
            for (int j=0;j<14;++j) if (z3_on_segment_strict(Q.poly[j], a1, a2)) splits.push_back(Q.poly[j]);
            if (splits.size() > 2) {
                double dx = (a2.x.a - a1.x.a) + (a2.x.b - a1.x.b) * SQRT3;
                double dy = (a2.y.a - a1.y.a) + (a2.y.b - a1.y.b) * SQRT3;
                std::sort(splits.begin(), splits.end(), [&](const Z3Pt& u, const Z3Pt& v) {
                    double du = (u.x.a + u.x.b * SQRT3) * dx + (u.y.a + u.y.b * SQRT3) * dy;
                    double dv = (v.x.a + v.x.b * SQRT3) * dx + (v.y.a + v.y.b * SQRT3) * dy;
                    return du < dv;
                });
            }
            for (size_t s=0; s+1<splits.size(); ++s) {
                Z3Pt mid2{{splits[s].x.a + splits[s+1].x.a, splits[s].x.b + splits[s+1].x.b},
                          {splits[s].y.a + splits[s+1].y.a, splits[s].y.b + splits[s+1].y.b}};
                if (z3_point_in_poly(mid2, Q.poly2) == 1) return true;
            }
        }
        return false;
    };
    return check_edges(A, B) || check_edges(B, A);
}

static bool tile_inside_cavity_exact(
    const TileGeom& tg,
    const std::vector<Z3Pt>& cav,
    const std::vector<Z3Pt>& cav2,
    double cx0, double cy0, double cx1, double cy1,
    const std::unordered_set<DirEdge, DirEdgeHash>& rev_bd
) {
    if (tg.x0 < cx0 - 1e-6 || tg.y0 < cy0 - 1e-6 || tg.x1 > cx1 + 1e-6 || tg.y1 > cy1 + 1e-6) return false;
    for (int i=0;i<14;++i) if (rev_bd.count(tg.edges[i])) return false;
    for (int i=0;i<14;++i) {
        if (!has_pt(cav, tg.poly[i]) && z3_point_in_poly(tg.poly[i], cav) == -1) return false;
    }
    int nC = int(cav.size());
    for (int i=0;i<14;++i) {
        const Z3Pt& a1 = tg.poly[i]; const Z3Pt& a2 = tg.poly[(i+1)%14];
        for (int j=0;j<nC;++j) {
            if (z3_segments_properly_cross(a1, a2, cav[j], cav[(j+1)%nC])) return false;
        }
    }
    std::vector<Z3Pt> splits;
    splits.reserve(6);
    for (int i=0;i<14;++i) {
        const Z3Pt& a1 = tg.poly[i]; const Z3Pt& a2 = tg.poly[(i+1)%14];
        splits.clear();
        splits.push_back(a1); splits.push_back(a2);
        for (int j=0;j<nC;++j) if (z3_on_segment_strict(cav[j], a1, a2)) splits.push_back(cav[j]);
        if (splits.size() > 2) {
            double dx = (a2.x.a - a1.x.a) + (a2.x.b - a1.x.b) * SQRT3;
            double dy = (a2.y.a - a1.y.a) + (a2.y.b - a1.y.b) * SQRT3;
            std::sort(splits.begin(), splits.end(), [&](const Z3Pt& u, const Z3Pt& v) {
                double du = (u.x.a + u.x.b * SQRT3) * dx + (u.y.a + u.y.b * SQRT3) * dy;
                double dv = (v.x.a + v.x.b * SQRT3) * dx + (v.y.a + v.y.b * SQRT3) * dy;
                return du < dv;
            });
        }
        for (size_t s=0; s+1<splits.size(); ++s) {
            Z3Pt mid2{{splits[s].x.a + splits[s+1].x.a, splits[s].x.b + splits[s+1].x.b},
                      {splits[s].y.a + splits[s+1].y.a, splits[s].y.b + splits[s+1].y.b}};
            if (z3_point_in_poly(mid2, cav2) == -1) return false;
        }
    }
    return true;
}

// ============================================================================
// MODE "mrv": Unrestricted Combinatorial Exact MRV-Edge Solver
// ============================================================================
static int run_mrv_mode() {
    int allow_reflect, forbid_orig, target_k, n_bd;
    uint64_t max_nodes;
    if (!(std::cin >> allow_reflect >> forbid_orig >> target_k >> max_nodes >> n_bd)) return 1;
    std::vector<Z3Pt> cav(n_bd), cav2(n_bd);
    std::unordered_set<DirEdge, DirEdgeHash> bd_edges, rev_bd;
    double cx0 = 1e18, cy0 = 1e18, cx1 = -1e18, cy1 = -1e18;
    std::vector<Z4Vec> cyc_z4(n_bd);
    for (int i=0;i<n_bd;++i) {
        std::cin >> cyc_z4[i].v[0] >> cyc_z4[i].v[1] >> cyc_z4[i].v[2] >> cyc_z4[i].v[3];
        cav[i] = z4_to_z3pt(cyc_z4[i]);
        cav2[i] = {{2 * cav[i].x.a, 2 * cav[i].x.b}, {2 * cav[i].y.a, 2 * cav[i].y.b}};
        auto [fx, fy] = z4_to_f2(cyc_z4[i]);
        cx0 = std::min(cx0, fx); cy0 = std::min(cy0, fy);
        cx1 = std::max(cx1, fx); cy1 = std::max(cy1, fy);
    }
    for (int i=0;i<n_bd;++i) {
        Z4Vec u = cyc_z4[i], v = cyc_z4[(i+1)%n_bd];
        bd_edges.insert({u, v});
        rev_bd.insert({v, u});
    }
    std::set<PlacementKey> orig_keys;
    for (int i=0;i<target_k;++i) {
        PlacementKey pk;
        std::cin >> pk.orient >> pk.shift.v[0] >> pk.shift.v[1] >> pk.shift.v[2] >> pk.shift.v[3];
        orig_keys.insert(pk);
    }

    // Pool of accepted TileGeom objects indexed by integer id
    std::vector<TileGeom> pool;
    std::unordered_map<PlacementKey, int, PlacementKeyHash> key_to_id; // -1 means outside cavity
    std::unordered_map<DirEdge, std::vector<int>, DirEdgeHash> edge_cands;
    // Overlap cache between pool ids
    std::unordered_map<uint64_t, bool> ov_cache;

    auto cands_for_edge = [&](const DirEdge& e) -> const std::vector<int>& {
        auto it = edge_cands.find(e);
        if (it != edge_cands.end()) return it->second;
        Z4Vec dv{};
        for (int c=0;c<4;++c) dv.v[c] = e.v.v[c] - e.u.v[c];
        int d = UNIT_DIR_MAP.at(dv);
        std::vector<int> out;
        for (auto [o, j] : EDGES_BY_DIR[allow_reflect ? 1 : 0][d]) {
            Z4Vec sh{};
            for (int c=0;c<4;++c) sh.v[c] = e.u.v[c] - ORIENT_VERTS[o][j].v[c];
            PlacementKey pk{o, sh};
            auto kit = key_to_id.find(pk);
            int id;
            if (kit == key_to_id.end()) {
                TileGeom tg = make_tile(o, sh);
                if (tile_inside_cavity_exact(tg, cav, cav2, cx0, cy0, cx1, cy1, rev_bd)) {
                    id = int(pool.size());
                    pool.push_back(std::move(tg));
                } else {
                    id = -1;
                }
                key_to_id[pk] = id;
            } else {
                id = kit->second;
            }
            if (id >= 0) out.push_back(id);
        }
        auto [ins, _] = edge_cands.emplace(e, std::move(out));
        return ins->second;
    };

    auto overlaps_any = [&](int id, const std::vector<int>& placed) -> bool {
        for (int pid : placed) {
            uint64_t a = std::min(id, pid), b = std::max(id, pid);
            uint64_t pk = (a << 32) | b;
            auto it = ov_cache.find(pk);
            bool ov;
            if (it == ov_cache.end()) {
                ov = tiles_overlap_exact(pool[id], pool[pid]);
                ov_cache[pk] = ov;
            } else {
                ov = it->second;
            }
            if (ov) return true;
        }
        return false;
    };

    uint64_t nodes = 0;
    bool aborted = false;
    std::vector<int> solution;
    std::unordered_set<DirEdge, DirEdgeHash> active = bd_edges;
    std::vector<int> placed;

    auto dfs = [&](auto&& self) -> bool {
        ++nodes;
        if (nodes > max_nodes) { aborted = true; return false; }
        if (active.empty()) {
            if (int(placed.size()) != target_k) return false;
            if (forbid_orig) {
                std::set<PlacementKey> pk;
                for (int id : placed) pk.insert(pool[id].key);
                if (pk == orig_keys) return false;
            }
            solution = placed;
            return true;
        }
        if (int(placed.size()) >= target_k) return false;

        std::vector<int> best_cands;
        bool have_best = false;
        for (const DirEdge& e : active) {
            const auto& cs = cands_for_edge(e);
            std::vector<int> valid;
            for (int id : cs) {
                const TileGeom& tg = pool[id];
                bool bad = false;
                for (int j=0;j<14;++j) {
                    if (active.count({tg.edges[j].v, tg.edges[j].u})) { bad = true; break; }
                }
                if (bad) continue;
                if (overlaps_any(id, placed)) continue;
                valid.push_back(id);
                if (have_best && valid.size() >= best_cands.size()) break;
            }
            if (valid.empty()) return false;
            if (!have_best || valid.size() < best_cands.size()) {
                best_cands = std::move(valid);
                have_best = true;
                if (best_cands.size() == 1) break;
            }
        }

        for (int id : best_cands) {
            const TileGeom& tg = pool[id];
            std::vector<DirEdge> removed_edges, added_edges;
            for (int j=0;j<14;++j) {
                const DirEdge& de = tg.edges[j];
                if (active.erase(de)) {
                    removed_edges.push_back(de);
                } else {
                    DirEdge rev{de.v, de.u};
                    active.insert(rev);
                    added_edges.push_back(rev);
                }
            }
            placed.push_back(id);
            if (self(self)) return true;
            placed.pop_back();
            for (const auto& de : added_edges) active.erase(de);
            for (const auto& de : removed_edges) active.insert(de);
            if (aborted) return false;
        }
        return false;
    };

    bool found = dfs(dfs);
    if (found) {
        std::cout << "SAT " << nodes << " " << pool.size() << " " << solution.size() << "\n";
        for (int id : solution) {
            const auto& pk = pool[id].key;
            std::cout << pk.orient << " " << pk.shift.v[0] << " " << pk.shift.v[1] << " "
                      << pk.shift.v[2] << " " << pk.shift.v[3] << "\n";
        }
    } else if (aborted) {
        std::cout << "ABORTED " << nodes << " " << pool.size() << " 0\n";
    } else {
        std::cout << "UNSAT " << nodes << " " << pool.size() << " 0\n";
    }
    return 0;
}

// ============================================================================
// MODE "cnf": Fast OpenMP Exact Overlap + Stokes-Flux DIMACS CNF Emitter (Engine B!)
// ============================================================================
static int run_cnf_mode() {
    std::string cnf_path;
    int forbid_orig, n_bd, n_orig, n_cands;
    if (!(std::cin >> cnf_path >> forbid_orig >> n_bd >> n_orig >> n_cands)) return 1;
    std::vector<Z3Pt> cav(n_bd), cav2(n_bd);
    std::unordered_set<DirEdge, DirEdgeHash> bd_edges, rev_bd;
    double cx0 = 1e18, cy0 = 1e18, cx1 = -1e18, cy1 = -1e18;
    std::vector<Z4Vec> cyc_z4(n_bd);
    for (int i=0;i<n_bd;++i) {
        std::cin >> cyc_z4[i].v[0] >> cyc_z4[i].v[1] >> cyc_z4[i].v[2] >> cyc_z4[i].v[3];
        cav[i] = z4_to_z3pt(cyc_z4[i]);
        cav2[i] = {{2 * cav[i].x.a, 2 * cav[i].x.b}, {2 * cav[i].y.a, 2 * cav[i].y.b}};
        auto [fx, fy] = z4_to_f2(cyc_z4[i]);
        cx0 = std::min(cx0, fx); cy0 = std::min(cy0, fy);
        cx1 = std::max(cx1, fx); cy1 = std::max(cy1, fy);
    }
    for (int i=0;i<n_bd;++i) {
        bd_edges.insert({cyc_z4[i], cyc_z4[(i+1)%n_bd]});
        rev_bd.insert({cyc_z4[(i+1)%n_bd], cyc_z4[i]});
    }
    std::set<PlacementKey> orig_keys;
    for (int i=0;i<n_orig;++i) {
        PlacementKey pk;
        std::cin >> pk.orient >> pk.shift.v[0] >> pk.shift.v[1] >> pk.shift.v[2] >> pk.shift.v[3];
        orig_keys.insert(pk);
    }
    std::vector<PlacementKey> raw_cands(n_cands);
    for (int i=0;i<n_cands;++i) {
        std::cin >> raw_cands[i].orient >> raw_cands[i].shift.v[0] >> raw_cands[i].shift.v[1]
                 >> raw_cands[i].shift.v[2] >> raw_cands[i].shift.v[3];
    }

    // Parallel exact inside-cavity filtering!
    std::vector<uint8_t> keep(n_cands, 0);
    std::vector<TileGeom> all_tg(n_cands);
    #pragma omp parallel for schedule(dynamic, 64)
    for (int i=0; i<n_cands; ++i) {
        TileGeom tg = make_tile(raw_cands[i].orient, raw_cands[i].shift);
        if (tile_inside_cavity_exact(tg, cav, cav2, cx0, cy0, cx1, cy1, rev_bd)) {
            keep[i] = 1;
            all_tg[i] = std::move(tg);
        }
    }

    std::vector<TileGeom> cands;
    std::unordered_map<PlacementKey, int, PlacementKeyHash> key_to_var;
    for (int i=0;i<n_cands;++i) {
        if (keep[i]) {
            int vid = int(cands.size()) + 1;
            key_to_var[all_tg[i].key] = vid;
            cands.push_back(std::move(all_tg[i]));
        }
    }
    int M = int(cands.size());

    // Directed edge -> variable list
    std::unordered_map<DirEdge, std::vector<int>, DirEdgeHash> edge_vars;
    for (int i=0;i<M;++i) {
        for (int j=0;j<14;++j) edge_vars[cands[i].edges[j]].push_back(i + 1);
    }

    // Spatial bucketed candidate pairs for parallel overlap checking!
    std::map<std::pair<int, int>, std::vector<int>> buckets;
    for (int i=0;i<M;++i) {
        int bx0 = int(std::floor(cands[i].x0 / 4.0)), bx1 = int(std::floor(cands[i].x1 / 4.0));
        int by0 = int(std::floor(cands[i].y0 / 4.0)), by1 = int(std::floor(cands[i].y1 / 4.0));
        for (int bx=bx0; bx<=bx1; ++bx) for (int by=by0; by<=by1; ++by) buckets[{bx, by}].push_back(i);
    }
    std::vector<std::pair<int, int>> pair_list;
    {
        std::unordered_set<uint64_t> seen;
        seen.reserve(size_t(M) * 40);
        for (const auto& [_, ids] : buckets) {
            for (size_t a=0; a<ids.size(); ++a) {
                for (size_t b=a+1; b<ids.size(); ++b) {
                    int ia = std::min(ids[a], ids[b]), ib = std::max(ids[a], ids[b]);
                    uint64_t k = (uint64_t(ia) << 32) | uint64_t(ib);
                    if (seen.insert(k).second) pair_list.push_back({ia, ib});
                }
            }
        }
    }
    std::vector<uint8_t> is_ov(pair_list.size(), 0);
    #pragma omp parallel for schedule(dynamic, 1024)
    for (size_t p=0; p<pair_list.size(); ++p) {
        if (tiles_overlap_exact(cands[pair_list[p].first], cands[pair_list[p].second])) is_ov[p] = 1;
    }

    // Count clauses and write DIMACS CNF
    std::vector<std::vector<int>> clauses;
    for (const auto& e : bd_edges) {
        const auto& vs = edge_vars[e];
        clauses.push_back(vs);
        for (size_t i=0;i<vs.size();++i) for (size_t j=i+1;j<vs.size();++j) clauses.push_back({-vs[i], -vs[j]});
    }
    for (const auto& [e, vs] : edge_vars) {
        if (bd_edges.count(e)) continue;
        for (size_t i=0;i<vs.size();++i) for (size_t j=i+1;j<vs.size();++j) clauses.push_back({-vs[i], -vs[j]});
        auto oit = edge_vars.find({e.v, e.u});
        for (int v : vs) {
            std::vector<int> cl;
            cl.push_back(-v);
            if (oit != edge_vars.end()) for (int ov : oit->second) cl.push_back(ov);
            clauses.push_back(std::move(cl));
        }
    }
    size_t n_ov = 0;
    for (size_t p=0; p<pair_list.size(); ++p) {
        if (is_ov[p]) {
            clauses.push_back({-(pair_list[p].first + 1), -(pair_list[p].second + 1)});
            ++n_ov;
        }
    }
    if (forbid_orig) {
        std::vector<int> blk;
        for (const auto& pk : orig_keys) {
            auto it = key_to_var.find(pk);
            if (it == key_to_var.end()) {
                std::cerr << "ERROR: original tile missing from Result-66 candidates!\n";
                return 3;
            }
            blk.push_back(-it->second);
        }
        clauses.push_back(std::move(blk));
    }

    std::ofstream out(cnf_path);
    out << "p cnf " << M << " " << clauses.size() << "\n";
    for (const auto& cl : clauses) {
        for (int lit : cl) out << lit << " ";
        out << "0\n";
    }
    out.close();

    // Print accepted candidate table (1-based) so Python can decode any SAT model
    std::cout << "OK " << M << " " << clauses.size() << " " << n_ov << "\n";
    for (int i=0;i<M;++i) {
        const auto& pk = cands[i].key;
        std::cout << pk.orient << " " << pk.shift.v[0] << " " << pk.shift.v[1] << " "
                  << pk.shift.v[2] << " " << pk.shift.v[3] << "\n";
    }
    return 0;
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    init_spectre_tables();
    std::string mode;
    if (!(std::cin >> mode)) return 1;
    if (mode == "mrv") return run_mrv_mode();
    if (mode == "cnf") return run_cnf_mode();
    return 2;
}
