TARGET=main
LATEX:=$(shell command -v tectonic 2>/dev/null || command -v pdflatex 2>/dev/null)
LATEXFLAGS=-interaction=nonstopmode -halt-on-error

.PHONY: all pdf clean

all: pdf

pdf:
ifndef LATEX
	$(error No LaTeX compiler found. Install tectonic or pdflatex first.)
endif
	@if echo "$(LATEX)" | grep -q "tectonic"; then \
		$(LATEX) $(TARGET).tex; \
	else \
		$(LATEX) $(LATEXFLAGS) $(TARGET).tex; \
		$(LATEX) $(LATEXFLAGS) $(TARGET).tex; \
	fi

clean:
	rm -f $(TARGET).aux $(TARGET).log $(TARGET).out $(TARGET).toc $(TARGET).pdf
