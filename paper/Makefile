# AIAgNav ICRA 2027 paper.  `make` builds, `make pages` is the number that matters.

TEX  := paper.tex
PDF  := $(TEX:.tex=.pdf)
LOG  := $(TEX:.tex=.log)
LIMIT := 8

.PHONY: all pages words budget clean watch

all: $(PDF)

$(PDF): $(TEX) refs.bib
	latexmk -pdf -interaction=nonstopmode -halt-on-error $(TEX)

# Page count, straight from the TeX log -- no extra tooling required.
pages: $(PDF)
	@n=$$(sed -n 's/.*Output written on .* (\([0-9]*\) page.*/\1/p' $(LOG) | tail -1); \
	echo "pages: $$n / $(LIMIT)"; \
	if [ "$$n" -gt "$(LIMIT)" ]; then \
	  echo "OVER THE LIMIT by $$((n - $(LIMIT))) page(s)"; exit 1; \
	else echo "within limit ($$(($(LIMIT) - $$n)) to spare)"; fi

# Per-section words vs. the budget measured from P-AgNav.
budget:
	@python3 budget.py $(TEX)

# Second opinion from texlive-extra-utils, if installed.
words:
	@texcount -inc -sum -sub=section $(TEX) 2>/dev/null || \
	  echo "texcount not installed; use 'make budget' instead"

watch:
	latexmk -pdf -pvc -interaction=nonstopmode $(TEX)

clean:
	latexmk -C 2>/dev/null || rm -f *.aux *.log *.out *.bbl *.blg *.fls *.fdb_latexmk
