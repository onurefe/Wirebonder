# Wirebonder Operator's Manual
#
#   make          build main.pdf
#   make watch    rebuild continuously on save
#   make todo     list every unwritten section
#   make clean    remove build artefacts (keeps the PDF)
#   make distclean remove build artefacts and the PDF

MAIN := main

.PHONY: all watch todo clean distclean

all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex product.tex wirebonder-manual.cls \
             $(wildcard chapters/*.tex) $(wildcard tables/*.tex)
	latexmk -lualatex -interaction=nonstopmode -halt-on-error $(MAIN).tex

watch:
	latexmk -lualatex -pvc -interaction=nonstopmode $(MAIN).tex

# Everything still to be written. Expected hits are the narrative chapters plus
# tables/specifications.tex (installation and consumables data, which the
# firmware cannot supply). The parameter, message and quick-reference material
# is complete and must stay that way.
todo:
	@grep -rn 'TODO' chapters/ tables/ || echo "No TODOs remaining."

clean:
	latexmk -c

distclean:
	latexmk -C
