#!/bin/bash
# Syntax check of root.tex without the IEEE class or the original images:
# swaps ieeeconf for article, draws images as boxes, drops the bibliography, runs pdflatex twice.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
T="$(mktemp -d)"
cp "$HERE"/sim_tables.tex "$T"/
mkdir -p "$T/images" && cp "$HERE"/images/*.png "$T/images/"
sed -e 's/^\\documentclass.*/\\documentclass[10pt,twocolumn]{article}\\usepackage[a4paper,margin=1.5cm]{geometry}\\PassOptionsToPackage{draft}{graphicx}/' \
    -e '/\\IEEEoverridecommandlockouts/d' -e '/\\overrideIEEEmargins/d' \
    -e '/\\bibliographystyle/d' -e '/^\\bibliography{/d' "$HERE/root.tex" > "$T/root.tex"
cd "$T"
pdflatex -interaction=nonstopmode root.tex > log1.txt 2>&1 || true
pdflatex -interaction=nonstopmode root.tex > log2.txt 2>&1 || true
grep -E "^!|Emergency|Missing \\$" log2.txt | grep -v "not found: usin" | grep . && { echo "LaTeX errors above (log in $T)"; exit 1; } || true
echo "OK: $(grep -o 'Output written.*' log2.txt)"; echo "undefined references:"; grep -E "Reference .* undefined" log2.txt | sort -u || true
echo "$T"
