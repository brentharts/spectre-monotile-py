// quantum_lanczos.cpp - Matrix-Free OpenMP C++20 Lanczos Eigensolver for the
// Spectre Pair-Qubit Frustrated Dipolar Transverse-Field Ising Model (quantgrav.pdf Result 17-18 / OP #15):
//
//   H(tau) = -Gamma * sum_{a=0}^{N-1} tau^x_a + sum_{0 <= a < b < N} K_{ab} tau^z_a tau^z_b
//
// Exploits exact Z2 spin-flip parity symmetry:
//   Even basis: |e_x> = (|x> + |x ^ full>) / sqrt(2),  x in [0, D),  D = 2^{N-1}
//   Odd  basis: |o_x> = (|x> - |x ^ full>) / sqrt(2),  x in [0, D),  D = 2^{N-1}
//
// In these bases:
//   - Diagonal E_diag[x] is identical for even and odd sectors (computed in O(D) via Gray code).
//   - tau^x_a (a < N-1) maps x -> x ^ (1 << a) with sign +1.
//   - tau^x_{N-1} maps x -> x ^ (D - 1) with sign +1 (even) or -1 (odd).
//
// Uses full reorthogonalized Lanczos (exact Jacobi eigensolver on tridiagonal T_m)
// to obtain machine-precision (<1e-11) E0_even, E1_even, E0_odd, and |psi0_even>,
// then computes exact:
//   - gap_even   = E1_even - E0_even
//   - gap_tunnel = E0_odd  - E0_even
//   - m_x        = (1/N) sum_a <tau^x_a>
//   - chi_SG     = (1/N) sum_{a,b} <tau^z_a tau^z_b>^2  (Edwards-Anderson spin-glass susceptibility)
//   - q_EA2      = (1/(N(N-1))) sum_{a!=b} <tau^z_a tau^z_b>^2
//   - Exact C_max, CHSH_max across ALL (or top-K) pairs via exact 5-correlator reduced rho_ab!
//
// Compile: g++ -O3 -march=native -fopenmp -std=c++20 quantum_lanczos.cpp -o bin/quantum_lanczos

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <omp.h>
#include <random>
#include <string>
#include <utility>
#include <vector>

// Exact symmetric tridiagonal / dense Jacobi eigensolver for small m x m matrix (m <= 160)
static void jacobi_eigh(int m, std::vector<double>& A, std::vector<double>& evals, std::vector<double>& V) {
    V.assign(m * m, 0.0);
    for (int i = 0; i < m; ++i) V[i * m + i] = 1.0;

    for (int sweep = 0; sweep < 100; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < m; ++p)
            for (int q = p + 1; q < m; ++q)
                off += A[p * m + q] * A[p * m + q];
        if (off < 1e-28) break;
        for (int p = 0; p < m; ++p) {
            for (int q = p + 1; q < m; ++q) {
                double apq = A[p * m + q];
                if (std::abs(apq) < 1e-16) continue;
                double app = A[p * m + p];
                double aqq = A[q * m + q];
                double tau = (aqq - app) / (2.0 * apq);
                double t = (tau >= 0.0) ? (1.0 / (tau + std::sqrt(1.0 + tau * tau)))
                                        : (-1.0 / (-tau + std::sqrt(1.0 + tau * tau)));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;
                A[p * m + p] = app - t * apq;
                A[q * m + q] = aqq + t * apq;
                A[p * m + q] = A[q * m + p] = 0.0;
                for (int r = 0; r < m; ++r) {
                    if (r == p || r == q) continue;
                    double arp = A[r * m + p];
                    double arq = A[r * m + q];
                    A[r * m + p] = A[p * m + r] = c * arp - s * arq;
                    A[r * m + q] = A[q * m + r] = s * arp + c * arq;
                }
                for (int r = 0; r < m; ++r) {
                    double vrp = V[r * m + p];
                    double vrq = V[r * m + q];
                    V[r * m + p] = c * vrp - s * vrq;
                    V[r * m + q] = s * vrp + c * vrq;
                }
            }
        }
    }
    evals.resize(m);
    for (int i = 0; i < m; ++i) evals[i] = A[i * m + i];
    // Sort ascending
    std::vector<int> idx(m);
    for (int i = 0; i < m; ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return evals[a] < evals[b]; });
    std::vector<double> se(m), sV(m * m);
    for (int k = 0; k < m; ++k) {
        se[k] = evals[idx[k]];
        for (int r = 0; r < m; ++r) sV[r * m + k] = V[r * m + idx[k]];
    }
    evals.swap(se);
    V.swap(sV);
}

// Compute diagonal energies E_diag[x] for all x in [0, D), D = 2^(N-1) (bit N-1 is 0 -> s_{N-1}=+1)
static void build_diag(int N, const std::vector<double>& K, std::vector<double>& diag) {
    const uint64_t D = 1ULL << (N - 1);
    diag.resize(D);

    // Split D into parallel chunks; each chunk initializes its starting Gray state and sweeps!
    const int nchunks = std::min<uint64_t>(D, 256);
    const uint64_t chunk_sz = (D + nchunks - 1) / nchunks;

    #pragma omp parallel for schedule(static)
    for (int ch = 0; ch < nchunks; ++ch) {
        uint64_t lo = (uint64_t)ch * chunk_sz;
        uint64_t hi = std::min(D, lo + chunk_sz);
        if (lo >= hi) continue;

        // Compute starting state lo in Gray code: g = x ^ (x >> 1)
        uint64_t g = lo ^ (lo >> 1);
        double s[32];
        for (int a = 0; a < N - 1; ++a) s[a] = ((g >> a) & 1ULL) ? -1.0 : 1.0;
        s[N - 1] = 1.0;

        double E = 0.0;
        for (int a = 0; a < N; ++a) {
            double sa = s[a];
            for (int b = a + 1; b < N; ++b) E += K[a * N + b] * sa * s[b];
        }
        diag[g] = E;

        for (uint64_t x = lo + 1; x < hi; ++x) {
            int bit = __builtin_ctzll(x);
            double sold = s[bit];
            double dE = 0.0;
            const double* Krow = &K[bit * N];
            for (int b = 0; b < N; ++b) dE += Krow[b] * s[b];
            // K[bit,bit] == 0 so dE = sum_{b!=bit} K[bit,b] s[b]
            E -= 2.0 * sold * dE;
            s[bit] = -sold;
            g ^= (1ULL << bit);
            diag[g] = E;
        }
    }
}

// Matrix-vector product y = H_sector * v
// parity_sign = +1.0 for even sector, -1.0 for odd sector
static void matvec(
    int N, double Gamma, double parity_sign,
    const std::vector<double>& diag,
    const std::vector<double>& v,
    std::vector<double>& y
) {
    const uint64_t D = 1ULL << (N - 1);
    const uint64_t mask = D - 1;
    const double negG = -Gamma;
    const double lastG = -Gamma * parity_sign;

    #pragma omp parallel for schedule(static) if(D >= 262144)
    for (uint64_t x = 0; x < D; ++x) {
        double acc = diag[x] * v[x];
        double sum_x = 0.0;
        for (int a = 0; a < N - 1; ++a) {
            sum_x += v[x ^ (1ULL << a)];
        }
        acc += negG * sum_x + lastG * v[x ^ mask];
        y[x] = acc;
    }
}

struct LanczosRes {
    double E0;
    double E1;
    int steps;
    double res0;
    double res1;
    std::vector<double> psi0;
};

static LanczosRes run_lanczos(
    int N, double Gamma, double parity_sign,
    const std::vector<double>& diag,
    int max_steps, bool want_psi0, bool want_E1, uint64_t seed
) {
    const uint64_t D = 1ULL << (N - 1);
    int M = (int)std::min<uint64_t>(D, (uint64_t)max_steps);

    // Store Lanczos basis Q[k] for full reorthogonalization & Ritz reconstruction
    // At N=24 (D=8.39M), 60 vectors = 4.0 GB (easily fits in 62 GB RAM!)
    std::vector<std::vector<double>> Q;
    Q.reserve(M + 1);

    std::vector<double> q0(D);
    // Initialize with deterministic smooth-random + classical-ground-biased start so both
    // paramagnetic and deeply frozen (Gamma -> 0) ground states converge rapidly!
    uint64_t best_x = 0;
    double min_d = diag[0];
    for (uint64_t x = 1; x < D; ++x) {
        if (diag[x] < min_d) { min_d = diag[x]; best_x = x; }
    }
    const bool use_omp = (D >= 262144);
    #pragma omp parallel if(use_omp)
    {
        std::mt19937_64 rng(seed + 1315423911ULL * (uint64_t)(omp_get_thread_num() + 1));
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        #pragma omp for schedule(static)
        for (uint64_t x = 0; x < D; ++x) {
            q0[x] = 0.35 + 0.15 * dist(rng);
        }
    }
    q0[best_x] += 1.0;
    double nrm = 0.0;
    #pragma omp parallel for reduction(+:nrm) schedule(static) if(use_omp)
    for (uint64_t x = 0; x < D; ++x) nrm += q0[x] * q0[x];
    nrm = std::sqrt(nrm);
    #pragma omp parallel for schedule(static) if(use_omp)
    for (uint64_t x = 0; x < D; ++x) q0[x] /= nrm;
    Q.push_back(std::move(q0));

    std::vector<double> alpha, beta;
    std::vector<double> w(D);
    double E0 = 0.0, E1 = 0.0, r0 = 1.0, r1 = 1.0;
    std::vector<double> last_V;
    int m_done = 0;
    int max_t = use_omp ? omp_get_max_threads() : 1;
    std::vector<std::vector<double>> loc_dots(max_t, std::vector<double>(M + 1, 0.0));
    std::vector<double> dots(M + 1, 0.0);

    for (int k = 0; k < M; ++k) {
        matvec(N, Gamma, parity_sign, diag, Q[k], w);
        double ak = 0.0;
        const double* qk = Q[k].data();
        const double* qprev = (k > 0) ? Q[k - 1].data() : nullptr;
        double bk_prev = (k > 0) ? beta[k - 1] : 0.0;
        #pragma omp parallel for reduction(+:ak) schedule(static) if(use_omp)
        for (uint64_t x = 0; x < D; ++x) {
            if (qprev) w[x] -= bk_prev * qprev[x];
            ak += qk[x] * w[x];
        }
        alpha.push_back(ak);

        // Single-pass fused Gram-Schmidt reorthogonalization (2 passes, only 2 OMP regions instead of 2*k!)
        for (int pass = 0; pass < 2; ++pass) {
            if (pass == 0) {
                #pragma omp parallel for schedule(static) if(use_omp)
                for (uint64_t x = 0; x < D; ++x) w[x] -= ak * qk[x];
            }
            std::fill(dots.begin(), dots.begin() + k + 1, 0.0);
            if (use_omp) {
                #pragma omp parallel
                {
                    int tid = omp_get_thread_num();
                    double* ld = loc_dots[tid].data();
                    for (int j = 0; j <= k; ++j) ld[j] = 0.0;
                    #pragma omp for schedule(static)
                    for (uint64_t x = 0; x < D; ++x) {
                        double wx = w[x];
                        for (int j = 0; j <= k; ++j) ld[j] += Q[j][x] * wx;
                    }
                }
                for (int tid = 0; tid < max_t; ++tid)
                    for (int j = 0; j <= k; ++j) dots[j] += loc_dots[tid][j];
                #pragma omp parallel for schedule(static)
                for (uint64_t x = 0; x < D; ++x) {
                    double sub = 0.0;
                    for (int j = 0; j <= k; ++j) sub += dots[j] * Q[j][x];
                    w[x] -= sub;
                }
            } else {
                for (int j = 0; j <= k; ++j) {
                    double dot = 0.0;
                    const double* qj = Q[j].data();
                    for (uint64_t x = 0; x < D; ++x) dot += qj[x] * w[x];
                    if (std::abs(dot) > 1e-16) {
                        for (uint64_t x = 0; x < D; ++x) w[x] -= dot * qj[x];
                    }
                }
            }
        }

        double bk = 0.0;
        #pragma omp parallel for reduction(+:bk) schedule(static) if(use_omp)
        for (uint64_t x = 0; x < D; ++x) bk += w[x] * w[x];
        bk = std::sqrt(bk);
        m_done = k + 1;

        bool check = (m_done >= 10 && (m_done % 5 == 0)) || (bk < 1e-13) || (m_done == M);
        if (check) {
            std::vector<double> T(m_done * m_done, 0.0), evals, V;
            for (int i = 0; i < m_done; ++i) {
                T[i * m_done + i] = alpha[i];
                if (i + 1 < m_done) T[i * m_done + (i + 1)] = T[(i + 1) * m_done + i] = beta[i];
            }
            jacobi_eigh(m_done, T, evals, V);
            E0 = evals[0];
            E1 = (m_done > 1) ? evals[1] : evals[0];
            r0 = bk * std::abs(V[(m_done - 1) * m_done + 0]);
            r1 = (m_done > 1) ? (bk * std::abs(V[(m_done - 1) * m_done + 1])) : 0.0;
            last_V.swap(V);
            if (bk < 1e-13 || (r0 < 1e-10 && (!want_E1 || r1 < 1e-10))) {
                break;
            }
        }
        beta.push_back(bk);
        if (k + 1 < M) {
            std::vector<double> qnext(D);
            double invb = 1.0 / bk;
            #pragma omp parallel for schedule(static) if(use_omp)
            for (uint64_t x = 0; x < D; ++x) qnext[x] = w[x] * invb;
            Q.push_back(std::move(qnext));
        }
    }

    LanczosRes res{E0, E1, m_done, r0, r1, {}};
    if (want_psi0) {
        res.psi0.assign(D, 0.0);
        #pragma omp parallel for schedule(static) if(use_omp)
        for (uint64_t x = 0; x < D; ++x) {
            double acc = 0.0;
            for (int j = 0; j < m_done; ++j) acc += last_V[j * m_done + 0] * Q[j][x];
            res.psi0[x] = acc;
        }
        double nn = 0.0;
        #pragma omp parallel for reduction(+:nn) schedule(static) if(use_omp)
        for (uint64_t x = 0; x < D; ++x) nn += res.psi0[x] * res.psi0[x];
        nn = 1.0 / std::sqrt(nn);
        #pragma omp parallel for schedule(static) if(use_omp)
        for (uint64_t x = 0; x < D; ++x) res.psi0[x] *= nn;
    }
    return res;
}

// Exact Wootters concurrence of a real Z2-even 2-qubit density matrix defined by:
//   xa = <Xa>, xb = <Xb>, xx = <Xa Xb>, yy = <Ya Yb>, zz = <Za Zb>
static double concurrence_from_corr(double xa, double xb, double xx, double yy, double zz) {
    // 4x4 real symmetric rho in {|00>, |01>, |10>, |11>} basis:
    //   rho = 0.25 * (I + xa X_a + xb X_b + xx X_a X_b + yy Y_a Y_b + zz Z_a Z_b)
    // Note: Y_a Y_b has (00,11) = -1, (01,10) = +1
    double rho[16] = {
        1 + zz,      xb,          xa,          xx - yy,
        xb,          1 - zz,      xx + yy,     xa,
        xa,          xx + yy,     1 - zz,      xb,
        xx - yy,     xa,          xb,          1 + zz
    };
    for (int i = 0; i < 16; ++i) rho[i] *= 0.25;

    // Spin-flipped rho_tilde = (Y x Y) rho* (Y x Y), where Y x Y permutes (0<->3, 1<->2) with signs (-1, +1, +1, -1)
    const int p[4] = {3, 2, 1, 0};
    const double sg[4] = {-1.0, 1.0, 1.0, -1.0};
    double rhot[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            rhot[i * 4 + j] = sg[i] * sg[j] * rho[p[i] * 4 + p[j]];

    // Compute sqrt(rho) via Jacobi eigendecomposition of symmetric 4x4 rho
    std::vector<double> A(rho, rho + 16), ev, V;
    jacobi_eigh(4, A, ev, V);
    double srho[16] = {0};
    for (int k = 0; k < 4; ++k) {
        double sv = (ev[k] > 0.0) ? std::sqrt(ev[k]) : 0.0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                srho[i * 4 + j] += sv * V[i * 4 + k] * V[j * 4 + k];
    }
    // Symmetric matrix M = sqrt(rho) * rhot * sqrt(rho) has the exact same eigenvalues as rho * rhot!
    double tmp[16] = {0}, Mmat[16] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                tmp[i * 4 + j] += srho[i * 4 + k] * rhot[k * 4 + j];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                Mmat[i * 4 + j] += tmp[i * 4 + k] * srho[k * 4 + j];
    std::vector<double> Mv(Mmat, Mmat + 16), mev, mV;
    jacobi_eigh(4, Mv, mev, mV);
    // mev is sorted ascending -> descending square roots:
    double l0 = std::sqrt(std::max(0.0, mev[3]));
    double l1 = std::sqrt(std::max(0.0, mev[2]));
    double l2 = std::sqrt(std::max(0.0, mev[1]));
    double l3 = std::sqrt(std::max(0.0, mev[0]));
    return std::max(0.0, l0 - l1 - l2 - l3);
}

static double chsh_from_corr(double xx, double yy, double zz) {
    // Diagonal T = diag(xx, yy, zz); sum of 2 largest squares:
    double a[3] = {xx * xx, yy * yy, zz * zz};
    std::sort(a, a + 3);
    return 2.0 * std::sqrt(std::max(0.0, a[1] + a[2]));
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    // Input format (supports batching multiple points in one process if desired):
    //   N Gamma max_steps max_ent_pairs
    //   N x N coupling matrix K (symmetric, zero diag)
    int N, max_steps, max_ent_pairs;
    double Gamma;
    while (std::cin >> N >> Gamma >> max_steps >> max_ent_pairs) {
        std::vector<double> K(N * N);
        for (int i = 0; i < N * N; ++i) std::cin >> K[i];

        double t0 = omp_get_wtime();
        std::vector<double> diag;
        build_diag(N, K, diag);

        // Even sector ground + first excited state
        auto ev_res = run_lanczos(N, Gamma, +1.0, diag, max_steps, true, true, 42ULL);
        // Odd sector ground state (gives Z2 tunneling splitting E0_odd - E0_even)
        auto od_res = run_lanczos(N, Gamma, -1.0, diag, max_steps, false, false, 137ULL);

        const uint64_t D = 1ULL << (N - 1);
        const uint64_t mask = D - 1;
        const auto& psi = ev_res.psi0;

        const bool use_omp_obs = (D >= 262144);
        // 1. Exact <X_a> for all a = 0..N-1
        std::vector<double> expX(N, 0.0);
        for (int a = 0; a < N; ++a) {
            uint64_t flip = (a < N - 1) ? (1ULL << a) : mask;
            double s = 0.0;
            #pragma omp parallel for reduction(+:s) schedule(static) if(use_omp_obs)
            for (uint64_t x = 0; x < D; ++x) s += psi[x] * psi[x ^ flip];
            expX[a] = s;
        }
        double mx = 0.0;
        for (int a = 0; a < N; ++a) mx += expX[a];
        mx /= N;

        // 2. Exact <Z_a Z_b> for ALL pairs (0 <= a < b < N) in ONE parallel pass!
        const int npairs = N * (N - 1) / 2;
        std::vector<std::pair<int, int>> all_pairs;
        all_pairs.reserve(npairs);
        for (int a = 0; a < N; ++a)
            for (int b = a + 1; b < N; ++b)
                all_pairs.push_back({a, b});

        std::vector<double> expZZ(npairs, 0.0);
        int nthreads = use_omp_obs ? omp_get_max_threads() : 1;
        std::vector<std::vector<double>> locZZ(nthreads, std::vector<double>(npairs, 0.0));
        #pragma omp parallel if(use_omp_obs)
        {
            int tid = use_omp_obs ? omp_get_thread_num() : 0;
            double* acc = locZZ[tid].data();
            double sg[32];
            sg[N - 1] = 1.0;
            #pragma omp for schedule(static)
            for (uint64_t x = 0; x < D; ++x) {
                double p = psi[x] * psi[x];
                if (p < 1e-30) continue;
                for (int a = 0; a < N - 1; ++a) sg[a] = ((x >> a) & 1ULL) ? -1.0 : 1.0;
                int pid = 0;
                for (int a = 0; a < N; ++a) {
                    double pa = p * sg[a];
                    for (int b = a + 1; b < N; ++b) {
                        acc[pid++] += pa * sg[b];
                    }
                }
            }
        }
        for (int tid = 0; tid < nthreads; ++tid)
            for (int pid = 0; pid < npairs; ++pid)
                expZZ[pid] += locZZ[tid][pid];

        double sum_zz2 = 0.0;
        for (int pid = 0; pid < npairs; ++pid) sum_zz2 += expZZ[pid] * expZZ[pid];
        double chi_SG = 1.0 + 2.0 * sum_zz2 / (double)N;
        double q_EA2 = sum_zz2 / (double)npairs;

        // 3. Exact <X_a X_b> and <Y_a Y_b>
        std::vector<int> eval_pids;
        if (max_ent_pairs <= 0 || npairs <= max_ent_pairs) {
            eval_pids.resize(npairs);
            for (int i = 0; i < npairs; ++i) eval_pids[i] = i;
        } else {
            eval_pids.resize(npairs);
            for (int i = 0; i < npairs; ++i) eval_pids[i] = i;
            std::partial_sort(eval_pids.begin(), eval_pids.begin() + max_ent_pairs, eval_pids.end(),
                              [&](int p1, int p2) {
                                  auto [a1, b1] = all_pairs[p1];
                                  auto [a2, b2] = all_pairs[p2];
                                  return std::abs(K[a1 * N + b1]) > std::abs(K[a2 * N + b2]);
                              });
            eval_pids.resize(max_ent_pairs);
        }

        double c_max = 0.0, chsh_max = 2.0;
        int best_ca = -1, best_cb = -1, best_sa = -1, best_sb = -1;
        for (int pid : eval_pids) {
            auto [a, b] = all_pairs[pid];
            uint64_t fa = (a < N - 1) ? (1ULL << a) : mask;
            uint64_t fb = (b < N - 1) ? (1ULL << b) : mask;
            uint64_t fab = fa ^ fb;
            double xx = 0.0, yy = 0.0;
            if (b < N - 1) {
                #pragma omp parallel for reduction(+:xx,yy) schedule(static) if(use_omp_obs)
                for (uint64_t x = 0; x < D; ++x) {
                    double prod = psi[x] * psi[x ^ fab];
                    xx += prod;
                    double sab = ((((x >> a) ^ (x >> b)) & 1ULL) ? -1.0 : 1.0);
                    yy -= prod * sab;
                }
            } else {
                #pragma omp parallel for reduction(+:xx,yy) schedule(static) if(use_omp_obs)
                for (uint64_t x = 0; x < D; ++x) {
                    double prod = psi[x] * psi[x ^ fab];
                    xx += prod;
                    double sa = (((x >> a) & 1ULL) ? -1.0 : 1.0);
                    yy -= prod * sa;
                }
            }
            double zz = expZZ[pid];
            double conc = concurrence_from_corr(expX[a], expX[b], xx, yy, zz);
            double chsh = chsh_from_corr(xx, yy, zz);
            if (conc > c_max) { c_max = conc; best_ca = a; best_cb = b; }
            if (chsh > chsh_max) { chsh_max = chsh; best_sa = a; best_sb = b; }
        }

        double dt = omp_get_wtime() - t0;
        std::cout << std::setprecision(12)
                  << "OK "
                  << ev_res.E0 << " "
                  << (ev_res.E1 - ev_res.E0) << " "
                  << (od_res.E0 - ev_res.E0) << " "
                  << mx << " "
                  << chi_SG << " "
                  << q_EA2 << " "
                  << c_max << " "
                  << chsh_max << " "
                  << ev_res.steps << " "
                  << ev_res.res0 << " "
                  << ev_res.res1 << " "
                  << best_ca << " " << best_cb << " "
                  << best_sa << " " << best_sb << " "
                  << dt << "\n" << std::flush;
    }
    return 0;
}
