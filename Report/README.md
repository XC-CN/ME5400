# Report

This folder contains a two-page conference-style LaTeX report for the ME5400 project.

## Files

- `main.tex`: paper source
- `Makefile`: minimal build helper

## Build

Run either of the following inside `Report/`:

```bash
make
```

or

```bash
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

## Notes

- The report references figures directly from `../Results/0020_results/`.
- The current workspace does not have a local LaTeX toolchain installed, so the `.tex` source was prepared but not compiled in this environment.
