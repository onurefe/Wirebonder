% wirebonder-manual.cls -- document class for the Wirebonder Operator's Manual.
%
% Derived from `book`. Exists to give the manual three recurring visual
% elements that must look identical everywhere they appear:
%
%   \keycap{SAVE}      a panel button name, drawn as a key
%   lcdscreen          a verbatim 20x4 LCD mockup in a fixed-width frame
%   note/warning/tip   callout boxes
%
% plus \TODO, which renders unwritten sections as a visible red block so they
% are obvious in the PDF and greppable before release.

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{wirebonder-manual}[2026/08/04 Wirebonder Operator's Manual]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
\ProcessOptions\relax
\LoadClass[11pt,oneside,openany]{book}

\RequirePackage[T1]{fontenc}
\RequirePackage{lmodern}
\RequirePackage[a4paper,margin=25mm,top=28mm,bottom=28mm]{geometry}
\RequirePackage{graphicx}
\RequirePackage{booktabs}
\RequirePackage{longtable}
\RequirePackage{array}
\RequirePackage{tabularx}
\RequirePackage{multirow}
\RequirePackage{xcolor}
\RequirePackage[most]{tcolorbox}
\RequirePackage{fancyhdr}
\RequirePackage{titlesec}
\RequirePackage{enumitem}
\RequirePackage{fancyvrb}
\RequirePackage{needspace}
\RequirePackage[hidelinks]{hyperref}

% ---------------------------------------------------------------- colours --

\definecolor{wbrule}{gray}{0.45}
\definecolor{wbkeybg}{gray}{0.92}
\definecolor{wbkeyline}{gray}{0.55}
\definecolor{wblcdbg}{gray}{0.95}
\definecolor{wbnote}{RGB}{0,90,150}
\definecolor{wbwarn}{RGB}{170,40,20}
\definecolor{wbtodo}{RGB}{200,0,0}

% ------------------------------------------------------- headers / footers --

\pagestyle{fancy}
\fancyhf{}
\renewcommand{\headrulewidth}{0.4pt}
\fancyhead[L]{\small\nouppercase{\leftmark}}
\fancyhead[R]{\small\productname}
\fancyfoot[C]{\small\thepage}
\fancypagestyle{plain}{\fancyhf{}\fancyfoot[C]{\small\thepage}%
  \renewcommand{\headrulewidth}{0pt}}

\renewcommand{\chaptermark}[1]{\markboth{\thechapter.\ #1}{}}

\titleformat{\chapter}[display]
  {\normalfont\huge\bfseries}
  {\normalsize\textcolor{wbrule}{CHAPTER \thechapter}}
  {6pt}{\Huge}
\titlespacing*{\chapter}{0pt}{0pt}{24pt}

% ------------------------------------------------------------- key legends --

% \keycap{SAVE} -- a panel button. Use for every physical button reference so
% the reader can always tell a button from a menu label.
\newcommand{\keycap}[1]{%
  \setlength{\fboxsep}{2pt}%
  \raisebox{-0.5pt}{\fcolorbox{wbkeyline}{wbkeybg}{\small\textsf{#1}}}%
}

% \menuitem{Loop Height} -- a label as it appears on the LCD.
\newcommand{\menuitem}[1]{\texttt{#1}}

% \msg{BUSY! TRY LATER} -- a message string as the machine displays it.
\newcommand{\msg}[1]{\texttt{#1}}

% \param{Search 1} -- a bonding parameter, hyperlinked to its reference row.
\newcommand{\param}[1]{\texttt{#1}}

% ------------------------------------------------------------ LCD mockups --

% 20x4 character display, drawn at its true aspect so screen mockups in the
% manual match what the operator sees. Body is verbatim.
\DefineVerbatimEnvironment{lcdverb}{Verbatim}{fontsize=\small,commandchars=\\\{\}}

% Width of one monospace character, used to size the LCD frame to 20 columns.
\newlength{\ttcharwidth}
\AtBeginDocument{\settowidth{\ttcharwidth}{\small\texttt{0}}}

\newtcolorbox{lcdscreen}[1][]{%
  enhanced, sharp corners, boxrule=0.8pt,
  colback=wblcdbg, colframe=wbrule,
  left=6pt, right=6pt, top=5pt, bottom=5pt,
  width=\dimexpr 22\ttcharwidth\relax,
  before skip=8pt, after skip=8pt,
  #1}

% --------------------------------------------------------------- callouts --

\newtcolorbox{wbnotebox}[1][Note]{%
  enhanced, sharp corners, boxrule=0pt, leftrule=3pt,
  colback=wbnote!4, colframe=wbnote,
  left=8pt, right=8pt, top=6pt, bottom=6pt,
  fonttitle=\bfseries\small, title={#1},
  attach title to upper=\par}

\newtcolorbox{wbwarnbox}[1][Warning]{%
  enhanced, sharp corners, boxrule=0pt, leftrule=3pt,
  colback=wbwarn!5, colframe=wbwarn,
  left=8pt, right=8pt, top=6pt, bottom=6pt,
  fonttitle=\bfseries\small, title={#1},
  attach title to upper=\par}

\newenvironment{wbnote}{\begin{wbnotebox}}{\end{wbnotebox}}
\newenvironment{wbwarning}{\begin{wbwarnbox}}{\end{wbwarnbox}}

% ------------------------------------------------------------------- TODO --

% Unwritten content. Deliberately loud: these must all be resolved or removed
% before the manual ships. Find them with:
%     grep -rn 'TODO' Docs/chapters/
\newtcolorbox{wbtodobox}{%
  enhanced, sharp corners, boxrule=1pt,
  colback=wbtodo!6, colframe=wbtodo,
  left=8pt, right=8pt, top=6pt, bottom=6pt,
  fonttitle=\bfseries\small, title={TO BE WRITTEN},
  attach title to upper=\par}

\newenvironment{wbtodo}{\begin{wbtodobox}\itshape}{\end{wbtodobox}}
\newcommand{\TODO}[1]{\begin{wbtodo}#1\end{wbtodo}}
\newcommand{\TODOinline}[1]{\textcolor{wbtodo}{\textbf{[#1]}}}

% Figure placeholder: a ruled box standing in for a photo or drawing.
\newcommand{\figureplaceholder}[2][0.55\textwidth]{%
  \begin{center}
  \fcolorbox{wbtodo}{wbtodo!4}{%
    \parbox[c][#1][c]{0.9\textwidth}{\centering\itshape
      \textcolor{wbtodo}{\textbf{FIGURE NEEDED}}\\[4pt] #2}}
  \end{center}}

% ----------------------------------------------------------------- tables --

% Right-aligned numeric column for parameter tables.
\newcolumntype{R}[1]{>{\raggedleft\arraybackslash}p{#1}}
\newcolumntype{L}[1]{>{\raggedright\arraybackslash}p{#1}}

% Provenance stamp for firmware-derived tables. Every generated table carries
% one so a reader (and a future editor) knows which source file to re-check
% when the firmware changes.
\newcommand{\derivedfrom}[1]{%
  \par\vspace{2pt}{\footnotesize\itshape\textcolor{wbrule}{Source: #1}}\par}
