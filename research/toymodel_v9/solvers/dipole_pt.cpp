// dipole_pt.cpp - High-precision Parallel Tempering (Replica Exchange) + Houdayer
// Isoenergetic Cluster Moves + 2-opt local polishing for the Spectre frustrated dipolar
// Ising model (quantgrav.pdf Result 17 / OP #15).
//
// Directly tests whether SA's 1-hit upper bounds at L4 N = 82, 150, 250, 400, 804
// were unconverged or close to the true ground state, running MULTIPLE INDEPENDENT
// Parallel-Tempering ensembles (each with R=32 geometric temperature ladders +
// Houdayer cluster moves + 2-opt quench) so we get genuine independent-run hit counts
// AND energy convergence statistics (best, median, worst, std across independent PT runs)!
//
// Compile: g++ -O3 -march=native -fopenmp -std=c++20 frontiers/dipole_pt.cpp -o bin/dipole_pt

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <omp.h>
#include <random>
#include <vector>

static double eval_E(int N, const std::vector<double>& W, const std::vector<int8_t>& s) {
    double E = 0.0;
    for (int i = 0; i < N; ++i) {
        double acc = 0.0;
        const double* row = &W[(size_t)i * N];
        for (int j = i + 1; j < N; ++j) acc += row[j] * s[j];
        E += s[i] * acc;
    }
    return E;
}

static void polish_1_2opt(int N, const std::vector<double>& W, std::vector<int8_t>& s, double& E) {
    std::vector<double> h(N, 0.0);
    for (int i = 0; i < N; ++i) {
        double acc = 0.0;
        const double* row = &W[(size_t)i * N];
        for (int j = 0; j < N; ++j) acc += row[j] * s[j];
        h[i] = acc;
    }
    bool improved = true;
    while (improved) {
        improved = false;
        // 1-opt greedy steepest descent
        for (int i = 0; i < N; ++i) {
            double dE = -2.0 * s[i] * h[i];
            if (dE < -1e-12) {
                double ds = -2.0 * s[i];
                s[i] = -s[i];
                E += dE;
                const double* row = &W[(size_t)i * N];
                for (int j = 0; j < N; ++j) h[j] += ds * row[j];
                improved = true;
            }
        }
        if (improved) continue;
        // 2-opt pair flips (exact!)
        for (int i = 0; i < N && !improved; ++i) {
            const double* ri = &W[(size_t)i * N];
            double si = s[i], hi = h[i];
            for (int j = i + 1; j < N; ++j) {
                double dE = -2.0 * si * hi - 2.0 * s[j] * h[j] + 4.0 * ri[j] * si * s[j];
                if (dE < -1e-12) {
                    double dsi = -2.0 * s[i], dsj = -2.0 * s[j];
                    s[i] = -s[i]; s[j] = -s[j];
                    E += dE;
                    const double* rj = &W[(size_t)j * N];
                    for (int k = 0; k < N; ++k) h[k] += dsi * ri[k] + dsj * rj[k];
                    improved = true;
                    break;
                }
            }
        }
    }
    E = eval_E(N, W, s);
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    // Input: N n_ensembles n_replicas n_pt_sweeps, followed by N*N W matrix
    int N, n_ens, R, n_sweeps;
    while (std::cin >> N >> n_ens >> R >> n_sweeps) {
        std::vector<double> W((size_t)N * N);
        double tot_abs = 0.0;
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                std::cin >> W[(size_t)i * N + j];
                if (j > i) tot_abs += std::abs(W[(size_t)i * N + j]);
            }
        }
        // Typical field scale
        double mean_h = 0.0;
        for (int i = 0; i < N; ++i) {
            double a = 0.0;
            for (int j = 0; j < N; ++j) a += std::abs(W[(size_t)i * N + j]);
            mean_h += a;
        }
        mean_h /= N;
        double T_hi = mean_h * 1.2;
        double T_lo = mean_h * 0.008;
        std::vector<double> beta(R);
        for (int r = 0; r < R; ++r) {
            double T = T_lo * std::pow(T_hi / T_lo, (double)r / std::max(1, R - 1));
            beta[r] = 1.0 / T;
        }

        // Precompute top-12 strongest neighbors per spin for Houdayer cluster moves
        int K_nb = std::min(N - 1, 12);
        std::vector<std::vector<int>> top_nb(N);
        for (int i = 0; i < N; ++i) {
            std::vector<int> idx;
            idx.reserve(N - 1);
            for (int j = 0; j < N; ++j) if (j != i) idx.push_back(j);
            std::partial_sort(idx.begin(), idx.begin() + K_nb, idx.end(), [&](int a, int b) {
                return std::abs(W[(size_t)i * N + a]) > std::abs(W[(size_t)i * N + b]);
            });
            idx.resize(K_nb);
            top_nb[i] = std::move(idx);
        }

        double t0 = omp_get_wtime();
        std::vector<double> ens_best_E(n_ens, 1e30);

        #pragma omp parallel for schedule(dynamic, 1)
        for (int e = 0; e < n_ens; ++e) {
            std::mt19937_64 rng(0x517cc1b727220a95ULL ^ ((uint64_t)(e + 1) * 0x9e3779b97f4a7c15ULL));
            std::uniform_real_distribution<double> U(0.0, 1.0);

            // Two paired replica ladders (A and B) for Houdayer isoenergetic cluster moves!
            std::vector<std::vector<int8_t>> sA(R, std::vector<int8_t>(N, 1));
            std::vector<std::vector<int8_t>> sB(R, std::vector<int8_t>(N, 1));
            std::vector<std::vector<double>> hA(R, std::vector<double>(N, 0.0));
            std::vector<std::vector<double>> hB(R, std::vector<double>(N, 0.0));
            std::vector<double> EA(R, 0.0), EB(R, 0.0);

            auto init_rep = [&](std::vector<int8_t>& s, std::vector<double>& h, double& E) {
                for (int i = 0; i < N; ++i) s[i] = (rng() & 1ULL) ? 1 : -1;
                E = 0.0;
                for (int i = 0; i < N; ++i) {
                    double acc = 0.0;
                    const double* row = &W[(size_t)i * N];
                    for (int j = 0; j < N; ++j) acc += row[j] * s[j];
                    h[i] = acc;
                    E += s[i] * acc;
                }
                E *= 0.5;
            };
            for (int r = 0; r < R; ++r) {
                init_rep(sA[r], hA[r], EA[r]);
                init_rep(sB[r], hB[r], EB[r]);
            }

            double best_E = 1e30;
            std::vector<int8_t> best_s(N, 1);

            auto mc_sweep = [&](std::vector<int8_t>& s, std::vector<double>& h, double& E, double b) {
                for (int i = 0; i < N; ++i) {
                    double dE = -2.0 * s[i] * h[i];
                    if (dE <= 0.0 || U(rng) < std::exp(-b * dE)) {
                        double ds = -2.0 * s[i];
                        s[i] = -s[i];
                        E += dE;
                        const double* row = &W[(size_t)i * N];
                        for (int j = 0; j < N; ++j) h[j] += ds * row[j];
                    }
                }
            };

            std::vector<uint8_t> vis(N);
            std::vector<int> q(N);

            for (int sw = 0; sw < n_sweeps; ++sw) {
                // 1. Single-spin Metropolis sweep on all replicas
                for (int r = 0; r < R; ++r) {
                    mc_sweep(sA[r], hA[r], EA[r], beta[r]);
                    mc_sweep(sB[r], hB[r], EB[r], beta[r]);
                }
                // 2. Parallel Tempering replica swaps (even/odd parity)
                int p = sw & 1;
                for (int r = p; r + 1 < R; r += 2) {
                    double dA = (beta[r + 1] - beta[r]) * (EA[r + 1] - EA[r]);
                    if (dA >= 0.0 || U(rng) < std::exp(dA)) {
                        sA[r].swap(sA[r + 1]); hA[r].swap(hA[r + 1]); std::swap(EA[r], EA[r + 1]);
                    }
                    double dB = (beta[r + 1] - beta[r]) * (EB[r + 1] - EB[r]);
                    if (dB >= 0.0 || U(rng) < std::exp(dB)) {
                        sB[r].swap(sB[r + 1]); hB[r].swap(hB[r + 1]); std::swap(EB[r], EB[r + 1]);
                    }
                }
                // 3. Houdayer / ICM isoenergetic cluster swap on cold half of ladder every 4 sweeps
                if ((sw & 3) == 0) {
                    for (int r = 0; r < R / 2; ++r) {
                        // Align overall Z2 sign randomly or by negative-overlap sites
                        int seed = (int)(rng() % N);
                        if (sA[r][seed] * sB[r][seed] > 0) continue;
                        std::fill(vis.begin(), vis.end(), 0);
                        int qh = 0, qt = 0;
                        vis[seed] = 1; q[qt++] = seed;
                        while (qh < qt) {
                            int u = q[qh++];
                            for (int v : top_nb[u]) {
                                if (!vis[v] && sA[r][v] * sB[r][v] < 0) {
                                    vis[v] = 1; q[qt++] = v;
                                }
                            }
                        }
                        if (qt > 1 && qt < N / 2) {
                            // Swap cluster spins between replica A and B at same temperature (Metropolis on exact E_A + E_B!)
                            // Compute exact new energies after swapping cluster spins
                            std::vector<int8_t> nA = sA[r], nB = sB[r];
                            for (int k = 0; k < qt; ++k) {
                                int u = q[k];
                                std::swap(nA[u], nB[u]);
                            }
                            double eA_new = eval_E(N, W, nA);
                            double eB_new = eval_E(N, W, nB);
                            double dEtot = (eA_new + eB_new) - (EA[r] + EB[r]);
                            if (dEtot <= 0.0 || U(rng) < std::exp(-beta[r] * dEtot)) {
                                sA[r].swap(nA); sB[r].swap(nB);
                                EA[r] = eA_new; EB[r] = eB_new;
                                // Refresh local fields hA, hB
                                for (int i = 0; i < N; ++i) {
                                    double aA = 0.0, aB = 0.0;
                                    const double* row = &W[(size_t)i * N];
                                    for (int j = 0; j < N; ++j) { aA += row[j] * sA[r][j]; aB += row[j] * sB[r][j]; }
                                    hA[r][i] = aA; hB[r][i] = aB;
                                }
                            }
                        }
                    }
                }
                // Track coldest replicas
                if (EA[0] < best_E) { best_E = EA[0]; best_s = sA[0]; }
                if (EB[0] < best_E) { best_E = EB[0]; best_s = sB[0]; }
            }
            // Final exact 1-opt + 2-opt polish on coldest replicas & best seen!
            polish_1_2opt(N, W, best_s, best_E);
            double e0A = EA[0], e0B = EB[0];
            polish_1_2opt(N, W, sA[0], e0A);
            polish_1_2opt(N, W, sB[0], e0B);
            best_E = std::min({best_E, e0A, e0B});
            ens_best_E[e] = best_E;
        }

        double dt = omp_get_wtime() - t0;
        std::sort(ens_best_E.begin(), ens_best_E.end());
        double global_E = ens_best_E[0];
        int hits = 0;
        for (double e : ens_best_E) if (std::abs(e - global_E) <= 1e-7) hits++;
        double med_E = ens_best_E[n_ens / 2];
        double worst_E = ens_best_E.back();
        double f_best = (global_E + tot_abs) / (2.0 * tot_abs);
        double f_med = (med_E + tot_abs) / (2.0 * tot_abs);
        double f_worst = (worst_E + tot_abs) / (2.0 * tot_abs);

        std::cout << std::setprecision(12)
                  << "PT_RES " << N << " " << n_ens << " " << hits
                  << " " << global_E << " " << f_best
                  << " " << f_med << " " << f_worst
                  << " " << dt << "\n" << std::flush;
    }
    return 0;
}
