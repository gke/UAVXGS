for f in *.md; do 
    pandoc "$f" -o "${f%.md}.pdf" \
        --pdf-engine=xelatex \
        --extract-media=. \
        -V geometry:margin=1in
done
