(TeX-add-style-hook
 "reflow-oven"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("report" "11pt" "twoside" "a4paper")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("cancel" "makeroom")))
   (TeX-run-style-hooks
    "latex2e"
    "report"
    "rep11"
    "fullpage"
    "hyperref"
    "mathtools"
    "booktabs"
    "ltablex"
    "amssymb"
    "graphicx"
    "tabularx"
    "gensymb"
    "pdfpages"
    "amsmath"
    "bm"
    "listings"
    "color"
    "float"
    "cancel")
   (TeX-add-symbols
    "csch"
    "dbar")
   (LaTeX-add-labels
    "fig:isp-power-switch"
    "fig:reflow-controller-sch"
    "fig:fast-pwm"
    "eq:pwm-freq"
    "tab:device-clocking-options"))
 :latex)

