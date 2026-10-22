# ------------------------------------------------------ curved spectre monotile
#
#   make               list the targets (same as `make help`)
#   make test          self tests: bezierpy.py, spectre_operator.py, spectre_fields.py, spectre_fem.py, spectre_curvature.py, spectre_optimize.py,
#                      spectre_patch.py, spectre_diffract.py,
#                      spectre_fractal.py, spectre_cosmology.py, chair44_unfold.py, chair44_slice.py, chair44_compare.py
#   make verify        check the symbolic (SymPy) curves against spectre.py
#   make check         test + verify
#   make figures       rebuild every figure in figures/
#   make pdf           build/symspectre.pdf (symbolic LaTeX document, needs pdflatex)
#   make show          open the interactive per-edge variations window
#   make show-fields   open the interactive distance-field window
#   make show-fem      open the interactive finite-element window (torsion, modes, heat)
#   make show-curvature open the interactive curvature window (comb, turning function)
#   make show-optimize open the interactive sensitivity / optimisation window
#   make show-patch    open the interactive patch / tiling-condition window
#   make show-diffract open the interactive form factor / diffraction window
#   make show-fractal  open the interactive fractal analysis window
#   make all           check + figures + pdf
#   make install       pip install -r requirements.txt
#   make install-min   pip install only numpy, scipy, matplotlib and sympy
#   make clean         remove build/ and Python caches
#
# The `bezier` package is optional: without it spectre.py falls back to the
# pure-Python bezierpy.py, so `make install-min` is enough for everything here.

PYTHON ?= python3

FIGDIR := figures
BUILD  := build
VARIATIONS_PNG := $(FIGDIR)/spectre_operator_variations.png
FIELDS_PNG     := $(FIGDIR)/spectre_distance_fields.png
FEM_PNG        := $(FIGDIR)/spectre_fem_fields.png
CURVATURE_PNG  := $(FIGDIR)/spectre_curvature.png
OPTIMIZE_PNG   := $(FIGDIR)/spectre_optimize.png
PATCH_PNG      := $(FIGDIR)/spectre_patch.png
DIFFRACT_PNG   := $(FIGDIR)/spectre_diffract.png
FRACTAL_PNG    := $(FIGDIR)/spectre_fractal.png
COSMOLOGY_PNG  := $(FIGDIR)/spectre_cosmology.png
UNFOLD_PNG     := $(FIGDIR)/chair44_unfold.png
SLICE_PNG      := $(FIGDIR)/chair44_slice.png
COMPARE_PNG    := $(FIGDIR)/chair44_compare.png

.DEFAULT_GOAL := help
.PHONY: help all check test verify figures pdf show show-fields show-fem show-curvature show-optimize show-patch show-diffract show-fractal install install-min clean

help:  ## list the targets
	@grep -hE '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "} {printf "  make %-12s %s\n", $$1, $$2}'

all: check figures pdf  ## check + figures + pdf

check: test verify  ## test + verify

test:  ## run the self tests (bezierpy, spectre_operator, spectre_fields, spectre_fem, spectre_curvature, spectre_optimize, spectre_patch, spectre_diffract, spectre_fractal, chair44_unfold, chair44_slice, chair44_compare)
	$(PYTHON) bezierpy.py
	$(PYTHON) spectre_operator.py --tests-only
	$(PYTHON) spectre_fields.py --tests-only
	$(PYTHON) spectre_fem.py --tests-only
	$(PYTHON) spectre_curvature.py --tests-only
	$(PYTHON) spectre_optimize.py --tests-only
	$(PYTHON) spectre_patch.py --tests-only
	$(PYTHON) spectre_diffract.py --tests-only
	$(PYTHON) spectre_fractal.py --tests-only
	$(PYTHON) spectre_cosmology.py --tests-only
	$(PYTHON) chair44_unfold.py --tests-only
	$(PYTHON) chair44_slice.py --tests-only
	$(PYTHON) chair44_compare.py --tests-only
	$(PYTHON) chair44_diffraction.py --tests-only
	$(PYTHON) spectre_orient.py --tests-only
	$(PYTHON) orientation_spectra.py --tests-only
	$(PYTHON) hyperuniformity.py --tests-only
	$(PYTHON) contact_operators.py --tests-only
	$(PYTHON) lift4d.py --tests-only
	$(PYTHON) lift4d_matrix.py --tests-only

verify:  ## symbolic (SymPy) curves vs the numeric spectre.py
	$(PYTHON) symspectre.py --verify --no-pdf --outdir $(BUILD)

figures:  ## rebuild every figure in figures/
	$(PYTHON) spectre_operator.py --skip-tests --no-show --save $(VARIATIONS_PNG)
	$(PYTHON) spectre_fields.py --skip-tests --no-show --save $(FIELDS_PNG)
	$(PYTHON) spectre_fem.py --skip-tests --no-show --save $(FEM_PNG)
	$(PYTHON) spectre_curvature.py --skip-tests --no-show --save $(CURVATURE_PNG)
	$(PYTHON) spectre_optimize.py --skip-tests --no-show --save $(OPTIMIZE_PNG)
	$(PYTHON) spectre_patch.py --skip-tests --no-show --save $(PATCH_PNG)
	$(PYTHON) spectre_diffract.py --skip-tests --no-show --save $(DIFFRACT_PNG)
	$(PYTHON) spectre_fractal.py --skip-tests --no-show --save $(FRACTAL_PNG)
	$(PYTHON) spectre_cosmology.py --skip-tests --no-show --save $(COSMOLOGY_PNG)
	$(PYTHON) chair44_unfold.py --survey 3000 --png $(UNFOLD_PNG)
	$(PYTHON) chair44_slice.py --png $(SLICE_PNG)
	$(PYTHON) chair44_compare.py --png $(COMPARE_PNG)
	$(PYTHON) lift4d.py --png $(FIGDIR)/lift4d.png

pdf:  ## build/symspectre.pdf, the symbolic LaTeX document (needs pdflatex)
	$(PYTHON) symspectre.py --outdir $(BUILD)

show:  ## run the tests, then open the interactive per-edge variations window
	$(PYTHON) spectre_operator.py

show-fields:  ## run the tests, then open the interactive distance-field window
	$(PYTHON) spectre_fields.py

show-fem:  ## run the tests, then open the interactive finite-element window
	$(PYTHON) spectre_fem.py

show-curvature:  ## run the tests, then open the interactive curvature window
	$(PYTHON) spectre_curvature.py

show-optimize:  ## run the tests, then open the interactive sensitivity / optimisation window
	$(PYTHON) spectre_optimize.py

show-patch:  ## run the tests, then open the interactive patch window
	$(PYTHON) spectre_patch.py

show-diffract:  ## run the tests, then open the interactive diffraction window
	$(PYTHON) spectre_diffract.py

show-fractal:  ## run the tests, then open the interactive fractal analysis window
	$(PYTHON) spectre_fractal.py

install:  ## pip install -r requirements.txt
	$(PYTHON) -m pip install -r requirements.txt

install-min:  ## pip install numpy scipy matplotlib sympy (bezier not needed)
	$(PYTHON) -m pip install numpy scipy matplotlib sympy

clean:  ## remove build/ and Python caches
	rm -rf $(BUILD) __pycache__
