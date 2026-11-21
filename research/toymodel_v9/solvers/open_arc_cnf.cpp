// open_arc_cnf.cpp - Fast OpenMP C++20 exact 1/2 Z[sqrt3] CNF generator for Frontier 1A:
// Open-Boundary Chiral Phason Worms & Critical Pinning-Arc Threshold on Level-4 Spectre!
//
// Compile: g++ -O3 -march=native -fopenmp -std=c++20 frontiers/open_arc_cnf.cpp -o bin/open_arc_cnf
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

static constexpr double SQRT3 = 1.7320508075688772935;
using i64 = int64_t;
using i128 = __int128_t;

struct Z3Num { i64 a, b; bool operator==(const Z3Num& o) const { return a==o.a && b==o.b; } };
struct Z3Pt { Z3Num x, y; bool operator==(const Z3Pt& o) const { return x==o.x && y==o.y; } };
struct Z4Vec {
    i64 v[4];
    bool operator==(const Z4Vec& o) const { return v[0]==o.v[0] && v[1]==o.v[1] && v[2]==o.v[2] && v[3]==o.v[3]; }
    bool operator<(const Z4Vec& o) const { for (int i=0;i<4;++i) if (v[i]!=o.v[i]) return v[i]<o.v[i]; return false; }
};
struct Z4Hash {
    size_t operator()(const Z4Vec& x) const noexcept {
        uint64_t h = 0x9e3779b97f4a7c15ULL;
        for (int i=0;i<4;++i) h ^= uint64_t(x.v[i]) + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
        return size_t(h);
    }
};
struct DirEdge {
    Z4Vec u, v;
    bool operator==(const DirEdge& o) const { return u==o.u && v==o.v; }
};
struct DirEdgeHash {
    size_t operator()(const DirEdge& e) const noexcept {
        Z4Hash zh; size_t h1 = zh(e.u), h2 = zh(e.v);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1<<6) + (h1>>2));
    }
};
struct PlacementKey {
    int orient; Z4Vec shift;
    bool operator==(const PlacementKey& o) const { return orient==o.orient && shift==o.shift; }
};
struct PlacementKeyHash {
    size_t operator()(const PlacementKey& k) const noexcept {
        size_t h = Z4Hash{}(k.shift);
        return h ^ (size_t(k.orient)*0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2));
    }
};

inline int z3_sign(i64 a, i64 b) {
    if (a==0 && b==0) return 0;
    if (a>=0 && b>=0) return 1;
    if (a<=0 && b<=0) return -1;
    i128 aa = i128(a)*i128(a), bb3 = i128(3)*i128(b)*i128(b);
    if (a>0) return (aa>bb3)?1:((aa<bb3)?-1:0);
    return (bb3>aa)?1:((bb3<aa)?-1:0);
}
inline int z3_cmp(Z3Num u, Z3Num v) { return z3_sign(u.a-v.a, u.b-v.b); }
inline int z3_orient(const Z3Pt& o, const Z3Pt& a, const Z3Pt& b) {
    i128 ax0=a.x.a-o.x.a, ax1=a.x.b-o.x.b, ay0=a.y.a-o.y.a, ay1=a.y.b-o.y.b;
    i128 bx0=b.x.a-o.x.a, bx1=b.x.b-o.x.b, by0=b.y.a-o.y.a, by1=b.y.b-o.y.b;
    i128 c0=(ax0*by0+3*ax1*by1)-(ay0*bx0+3*ay1*bx1);
    i128 c1=(ax0*by1+ax1*by0)-(ay0*bx1+ay1*bx0);
    if (c0==0 && c1==0) return 0;
    if (c0>=0 && c1>=0) return 1;
    if (c0<=0 && c1<=0) return -1;
    i128 lhs=c0*c0, rhs=i128(3)*c1*c1;
    if (c0>0) return (lhs>rhs)?1:((lhs<rhs)?-1:0);
    return (rhs>lhs)?1:((rhs<lhs)?-1:0);
}
inline bool z3_on_segment_strict(const Z3Pt& p, const Z3Pt& a, const Z3Pt& b) {
    if (p==a || p==b) return false;
    if (z3_orient(a,b,p)!=0) return false;
    return (z3_cmp(p.x,a.x)*z3_cmp(p.x,b.x)<=0) && (z3_cmp(p.y,a.y)*z3_cmp(p.y,b.y)<=0);
}
inline bool z3_segments_properly_cross(const Z3Pt& a, const Z3Pt& b, const Z3Pt& c, const Z3Pt& d) {
    int o1=z3_orient(a,b,c), o2=z3_orient(a,b,d);
    if (o1==0 || o2==0 || o1==o2) return false;
    int o3=z3_orient(c,d,a), o4=z3_orient(c,d,b);
    return o3!=0 && o4!=0 && o3!=o4;
}
inline Z3Pt z4_to_z3pt(const Z4Vec& v) { return {{2*v.v[0]+v.v[1], v.v[2]}, {v.v[2]+2*v.v[3], v.v[1]}}; }
inline std::pair<double,double> z4_to_f2(const Z4Vec& v) {
    return {v.v[0]+0.5*v.v[1]+0.5*SQRT3*v.v[2], 0.5*SQRT3*v.v[1]+0.5*v.v[2]+v.v[3]};
}
static int z3_point_in_poly(const Z3Pt& p, const std::vector<Z3Pt>& poly) {
    int n=int(poly.size()); bool inside=false;
    for (int i=0;i<n;++i) {
        const Z3Pt& a=poly[i], &b=poly[(i+1)%n];
        if (p==a || z3_on_segment_strict(p,a,b)) return 0;
        int ca=z3_cmp(a.y,p.y), cb=z3_cmp(b.y,p.y);
        if ((ca>0)!=(cb>0)) {
            if (cb>0) { if (z3_orient(a,b,p)>0) inside=!inside; }
            else { if (z3_orient(b,a,p)>0) inside=!inside; }
        }
    }
    return inside?1:-1;
}

struct TileGeom {
    PlacementKey key;
    std::array<Z4Vec,14> verts;
    std::array<DirEdge,14> edges;
    std::vector<Z3Pt> poly, poly2;
    double x0,y0,x1,y1, cx, cy;
};

static std::array<std::array<Z4Vec,14>,12> ORIENT_VERTS;
static void init_tables() {
    int MZ[4][4]={{0,0,0,-1},{0,0,1,1},{1,0,0,0},{0,1,0,0}};
    int ZP[12][4][4];
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) ZP[0][i][j]=(i==j);
    for (int k=1;k<12;++k) for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
        int s=0; for (int m=0;m<4;++m) s+=MZ[i][m]*ZP[k-1][m][j]; ZP[k][i][j]=s;
    }
    int edge_dirs[14]={0,10,1,3,0,2,5,7,4,6,6,8,11,9};
    std::array<Z4Vec,14> base; base[0]={{0,0,0,0}};
    for (int i=0;i<13;++i) for (int c=0;c<4;++c) base[i+1].v[c]=base[i].v[c]+ZP[edge_dirs[i]][c][0];
    for (int k=0;k<12;++k) for (int i=0;i<14;++i) {
        Z4Vec p{}; for (int r=0;r<4;++r) { i64 s=0; for (int c=0;c<4;++c) s+=ZP[k][r][c]*base[i].v[c]; p.v[r]=s; }
        ORIENT_VERTS[k][i]=p;
    }
}
static TileGeom make_tile(int o, const Z4Vec& sh) {
    TileGeom tg; tg.key={o,sh}; tg.poly.resize(14); tg.poly2.resize(14);
    tg.x0=tg.y0=1e18; tg.x1=tg.y1=-1e18; tg.cx=tg.cy=0.0;
    for (int i=0;i<14;++i) {
        for (int c=0;c<4;++c) tg.verts[i].v[c]=ORIENT_VERTS[o][i].v[c]+sh.v[c];
        tg.poly[i]=z4_to_z3pt(tg.verts[i]);
        tg.poly2[i]={{2*tg.poly[i].x.a,2*tg.poly[i].x.b},{2*tg.poly[i].y.a,2*tg.poly[i].y.b}};
        auto [fx,fy]=z4_to_f2(tg.verts[i]);
        tg.x0=std::min(tg.x0,fx); tg.y0=std::min(tg.y0,fy); tg.x1=std::max(tg.x1,fx); tg.y1=std::max(tg.y1,fy);
        tg.cx+=fx; tg.cy+=fy;
    }
    tg.cx /= 14.0; tg.cy /= 14.0;
    for (int i=0;i<14;++i) tg.edges[i]={tg.verts[i],tg.verts[(i+1)%14]};
    return tg;
}
static bool has_pt(const std::vector<Z3Pt>& poly, const Z3Pt& p) {
    for (const auto& q : poly) if (q==p) return true; return false;
}
static bool tiles_overlap_exact(const TileGeom& A, const TileGeom& B) {
    if (A.x1<B.x0-1e-6 || B.x1<A.x0-1e-6 || A.y1<B.y0-1e-6 || B.y1<A.y0-1e-6) return false;
    bool same=true; for (int i=0;i<14;++i) if (!has_pt(B.poly,A.poly[i])) { same=false; break; }
    if (same) return true;
    for (int i=0;i<14;++i) for (int j=0;j<14;++j)
        if (z3_segments_properly_cross(A.poly[i],A.poly[(i+1)%14],B.poly[j],B.poly[(j+1)%14])) return true;
    for (int i=0;i<14;++i) if (!has_pt(B.poly,A.poly[i]) && z3_point_in_poly(A.poly[i],B.poly)==1) return true;
    for (int j=0;j<14;++j) if (!has_pt(A.poly,B.poly[j]) && z3_point_in_poly(B.poly[j],A.poly)==1) return true;
    std::vector<Z3Pt> sp; sp.reserve(6);
    auto chk = [&](const TileGeom& P, const TileGeom& Q) {
        for (int i=0;i<14;++i) {
            const Z3Pt& a1=P.poly[i], &a2=P.poly[(i+1)%14];
            sp.clear(); sp.push_back(a1); sp.push_back(a2);
            for (int j=0;j<14;++j) if (z3_on_segment_strict(Q.poly[j],a1,a2)) sp.push_back(Q.poly[j]);
            if (sp.size()>2) {
                double dx=(a2.x.a-a1.x.a)+(a2.x.b-a1.x.b)*SQRT3, dy=(a2.y.a-a1.y.a)+(a2.y.b-a1.y.b)*SQRT3;
                std::sort(sp.begin(),sp.end(),[&](const Z3Pt& u, const Z3Pt& v){
                    return ((u.x.a+u.x.b*SQRT3)*dx+(u.y.a+u.y.b*SQRT3)*dy) < ((v.x.a+v.x.b*SQRT3)*dx+(v.y.a+v.y.b*SQRT3)*dy);
                });
            }
            for (size_t s=0;s+1<sp.size();++s) {
                Z3Pt mid2{{sp[s].x.a+sp[s+1].x.a,sp[s].x.b+sp[s+1].x.b},{sp[s].y.a+sp[s+1].y.a,sp[s].y.b+sp[s+1].y.b}};
                if (z3_point_in_poly(mid2,Q.poly2)==1) return true;
            }
        }
        return false;
    };
    return chk(A,B) || chk(B,A);
}

static bool pt_in_tile_f(double px, double py, const TileGeom& tg) {
    if (px < tg.x0 || px > tg.x1 || py < tg.y0 || py > tg.y1) return false;
    bool ins = false;
    for (int i = 0; i < 14; ++i) {
        auto [x1, y1] = z4_to_f2(tg.verts[i]);
        auto [x2, y2] = z4_to_f2(tg.verts[(i + 1) % 14]);
        if ((y1 > py) != (y2 > py)) {
            if (px < x1 + (py - y1) * (x2 - x1) / (y2 - y1)) ins = !ins;
        }
    }
    return ins;
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    init_tables();

    // Protocol:
    //   cnf_path
    //   n_cav n_pin n_bd_cyc n_raw
    //   n_cav lines: orient s0 s1 s2 s3
    //   n_pin lines: orient s0 s1 s2 s3
    //   n_bd_cyc lines: v0 v1 v2 v3 (CCW boundary cycle of cavity C)
    //   n_raw lines: orient s0 s1 s2 s3 (candidate Result-66 placements)
    std::string cnf_path;
    int n_cav, n_pin, n_bd, n_raw;
    if (!(std::cin >> cnf_path >> n_cav >> n_pin >> n_bd >> n_raw)) return 1;

    std::vector<TileGeom> cav(n_cav), pin(n_pin);
    std::unordered_set<PlacementKey, PlacementKeyHash> orig_set;
    std::unordered_set<DirEdge, DirEdgeHash> cav_edges, pin_edges;
    for (int i = 0; i < n_cav; ++i) {
        int o; Z4Vec s; std::cin >> o >> s.v[0] >> s.v[1] >> s.v[2] >> s.v[3];
        cav[i] = make_tile(o, s);
        orig_set.insert(cav[i].key);
        for (const auto& e : cav[i].edges) cav_edges.insert(e);
    }
    for (int i = 0; i < n_pin; ++i) {
        int o; Z4Vec s; std::cin >> o >> s.v[0] >> s.v[1] >> s.v[2] >> s.v[3];
        pin[i] = make_tile(o, s);
        for (const auto& e : pin[i].edges) pin_edges.insert(e);
    }
    std::vector<Z3Pt> cav_poly2(n_bd);
    for (int i = 0; i < n_bd; ++i) {
        Z4Vec v; std::cin >> v.v[0] >> v.v[1] >> v.v[2] >> v.v[3];
        Z3Pt p = z4_to_z3pt(v);
        cav_poly2[i] = {{2 * p.x.a, 2 * p.x.b}, {2 * p.y.a, 2 * p.y.b}};
    }
    std::vector<PlacementKey> raw(n_raw);
    for (int i = 0; i < n_raw; ++i) {
        std::cin >> raw[i].orient >> raw[i].shift.v[0] >> raw[i].shift.v[1] >> raw[i].shift.v[2] >> raw[i].shift.v[3];
    }

    // Split boundary edges of C into pinned vs open
    std::unordered_set<DirEdge, DirEdgeHash> bd_pinned, bd_all;
    for (const auto& e : cav_edges) {
        DirEdge rev{e.v, e.u};
        if (!cav_edges.count(rev)) {
            bd_all.insert(e);
            if (pin_edges.count(rev)) bd_pinned.insert(e);
        }
    }

    // Parallel filter of raw candidates:
    //   must overlap at least one tile in cav, and must NOT overlap any tile in pin!
    std::vector<uint8_t> keep(n_raw, 0);
    std::vector<TileGeom> raw_tg(n_raw);
    #pragma omp parallel for schedule(dynamic, 64)
    for (int i = 0; i < n_raw; ++i) {
        TileGeom tg = make_tile(raw[i].orient, raw[i].shift);
        bool hit_cav = false;
        for (int c = 0; c < n_cav; ++c) {
            if (tiles_overlap_exact(tg, cav[c])) { hit_cav = true; break; }
        }
        if (!hit_cav) continue;
        bool hit_pin = false;
        for (int p = 0; p < n_pin; ++p) {
            if (tiles_overlap_exact(tg, pin[p])) { hit_pin = true; break; }
        }
        if (!hit_pin) {
            keep[i] = 1;
            raw_tg[i] = std::move(tg);
        }
    }

    std::vector<TileGeom> cands;
    std::vector<int> orig_vars;
    for (int i = 0; i < n_raw; ++i) {
        if (keep[i]) {
            if (orig_set.count(raw_tg[i].key)) orig_vars.push_back((int)cands.size() + 1);
            cands.push_back(std::move(raw_tg[i]));
        }
    }
    int M = (int)cands.size();

    // Parallel pairwise overlap exclusion using bbox filter
    std::vector<std::vector<std::pair<int, int>>> loc_ov(omp_get_max_threads());
    #pragma omp parallel for schedule(dynamic, 16)
    for (int i = 0; i < M; ++i) {
        int tid = omp_get_thread_num();
        for (int j = i + 1; j < M; ++j) {
            if (tiles_overlap_exact(cands[i], cands[j])) {
                loc_ov[tid].push_back({-(i + 1), -(j + 1)});
            }
        }
    }

    // Edge map
    std::unordered_map<DirEdge, std::vector<int>, DirEdgeHash> edge_vars;
    edge_vars.reserve(M * 14);
    for (int i = 0; i < M; ++i) {
        for (const auto& e : cands[i].edges) edge_vars[e].push_back(i + 1);
    }

    // Collect all clauses
    std::vector<std::vector<int>> clauses;
    // 1. Blocking clause forbidding exact original cavity
    {
        std::vector<int> blk;
        for (int v : orig_vars) blk.push_back(-v);
        clauses.push_back(std::move(blk));
    }
    // 2. Pairwise overlaps
    size_t n_ov = 0;
    for (const auto& lv : loc_ov) {
        n_ov += lv.size();
        for (auto [a, b] : lv) clauses.push_back({a, b});
    }
    // 3a. Pinned boundary edges covered ONCE
    for (const auto& e : bd_pinned) {
        auto it = edge_vars.find(e);
        if (it == edge_vars.end() || it->second.empty()) {
            std::cout << "UNSAT_BD_GAP 0 0 " << bd_pinned.size() << " " << (bd_all.size() - bd_pinned.size()) << "\n";
            return 0;
        }
        const auto& vs = it->second;
        clauses.push_back(vs);
        for (size_t a = 0; a < vs.size(); ++a)
            for (size_t b = a + 1; b < vs.size(); ++b)
                clauses.push_back({-vs[a], -vs[b]});
    }
    // 3b. Exact Stokes interior-edge flux conservation for every directed edge whose midpoint is strictly inside C!
    std::vector<DirEdge> all_e_keys;
    all_e_keys.reserve(edge_vars.size());
    for (const auto& kv : edge_vars) all_e_keys.push_back(kv.first);

    std::vector<uint8_t> is_int_edge(all_e_keys.size(), 0);
    #pragma omp parallel for schedule(static)
    for (size_t idx = 0; idx < all_e_keys.size(); ++idx) {
        const auto& e = all_e_keys[idx];
        DirEdge rev{e.v, e.u};
        if (bd_all.count(e) || bd_all.count(rev)) continue;
        Z3Pt u3 = z4_to_z3pt(e.u), v3 = z4_to_z3pt(e.v);
        Z3Pt mid2{{u3.x.a + v3.x.a, u3.x.b + v3.x.b}, {u3.y.a + v3.y.a, u3.y.b + v3.y.b}};
        if (z3_point_in_poly(mid2, cav_poly2) == 1) is_int_edge[idx] = 1;
    }
    for (size_t idx = 0; idx < all_e_keys.size(); ++idx) {
        if (!is_int_edge[idx]) continue;
        const auto& e = all_e_keys[idx];
        const auto& vs = edge_vars[e];
        DirEdge rev{e.v, e.u};
        auto it = edge_vars.find(rev);
        for (int v : vs) {
            if (it != edge_vars.end() && !it->second.empty()) {
                std::vector<int> cl;
                cl.reserve(1 + it->second.size());
                cl.push_back(-v);
                for (int rv : it->second) cl.push_back(rv);
                clauses.push_back(std::move(cl));
            } else {
                clauses.push_back({-v});
            }
        }
    }
    // 3c. Every original tile centroid in C must be covered (no empty voids from open boundary!)
    for (int c = 0; c < n_cav; ++c) {
        std::vector<int> cov;
        for (int i = 0; i < M; ++i) {
            if (pt_in_tile_f(cav[c].cx, cav[c].cy, cands[i])) cov.push_back(i + 1);
        }
        clauses.push_back(std::move(cov));
    }

    // Write DIMACS CNF
    std::ofstream out(cnf_path);
    out << "p cnf " << M << " " << clauses.size() << "\n";
    for (const auto& cl : clauses) {
        for (int lit : cl) out << lit << " ";
        out << "0\n";
    }
    out.close();

    // Output header + candidate table
    std::cout << "OK " << M << " " << clauses.size() << " " << bd_pinned.size()
              << " " << (bd_all.size() - bd_pinned.size()) << " " << n_ov << "\n";
    for (int i = 0; i < M; ++i) {
        const auto& k = cands[i].key;
        std::cout << k.orient << " " << k.shift.v[0] << " " << k.shift.v[1]
                  << " " << k.shift.v[2] << " " << k.shift.v[3] << "\n";
    }
    return 0;
}
