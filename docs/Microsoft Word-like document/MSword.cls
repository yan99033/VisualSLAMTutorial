%
% I8,        8        ,8I                               88
% `8b       d8b       d8'                               88
%  "8,     ,8"8,     ,8"                                88
%   Y8     8P Y8     8P  ,adPPYba,  8b,dPPYba,  ,adPPYb,88
%   `8b   d8' `8b   d8' a8"     "8a 88P'   "Y8 a8"    `Y88
%    `8a a8'   `8a a8'  8b       d8 88         8b       88
%     `8a8'     `8a8'   "8a,   ,a8" 88         "8a,   ,d88
%      `8'       `8'     `"YbbdP"'  88          `"8bbdP"Y8
%
%---------------------------------------------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% *** LaTeX-class for Word-like document, 
%
%                           V.01.0 // Version 01, with 0 update(s) or patch(es)
%
% First released January 2023 
% Last updated January 2023.
%
%---------------------------------------------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   Defining the .cls file
%_______________________________________________________________
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{MSword}[2023/01/11 Word]
\LoadClass[a4paper]{extarticle} % Base class
\RequirePackage[utf8]{inputenc}
%................................................................
%
%                           Options
%_______________________________________________________________
% Math:
%-------------------------------
\newif\if@mathpackages
\DeclareOption{math}{\@mathpackagestrue}
%
% Hidelinks:
%-------------------------------
\newif\if@hidelinks
\DeclareOption{hidelinks}{\@hidelinkstrue}
%
% Wordlinks:
%-------------------------------
\newif\if@wordlinks
\DeclareOption{wordlinks}{\@wordlinkstrue}
%
% no bibliography (skip code):
%-------------------------------
\newif\if@nobib
\DeclareOption{nobib}{\@nobibtrue}
%
% Unknown options:
%-------------------------------
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{extarticle}}
\ProcessOptions\relax
%................................................................
%
%
%                           Packages
%_______________________________________________________________
\RequirePackage[T1]{fontenc}
\usepackage{tgtermes}               % times font
\RequirePackage{geometry}           % to adjust margins
\RequirePackage{xcolor}             % colours
\RequirePackage{graphicx}           % Easier to include pictures
\RequirePackage{hyperref}           % Clickable links
\RequirePackage{fancyhdr}           % Headers and footers on pages
\RequirePackage{lastpage}           % For referencing total page number.
\RequirePackage{multicol,caption}   % For mulicolumn document
%
\hypersetup{pdfborderstyle={/S/U/W 1}}
% Hide links option:
\if@hidelinks
    \hypersetup{hidelinks}
\fi
%................................................................
%
%                           Math
%_______________________________________________________________
\if@mathpackages
\RequirePackage{mathtools}
\RequirePackage{amssymb}
\fi
%................................................................
%
%                   Document adjustments
%_______________________________________________________________
\geometry{
    footskip=2cm,
    left=2cm,right=2cm,
    top=2cm,
    bottom=3cm,
    rmargin=2cm
}
%
% More "Word-like":
\setlength{\parindent}{2em}
\setlength{\parskip}{0.5em}
\renewcommand{\baselinestretch}{1.5}
%
% Wordcount function:
%   
\newcommand{\quickwordcount}[1]{%
  \immediate\write18{texcount -1 -merge -sum -q #1.tex > #1-words.sum}%
   \input{#1-words.sum}%
}
%................................................................
%
%                              Colors
%_______________________________________________________________
\definecolor{wordhyperlink}{HTML}{327FCC}
\definecolor{redtext}{HTML}{FF3737}
\definecolor{yellowhighlight}{HTML}{FFFF00}
\definecolor{graytext}{HTML}{C1C1C1}

\newcommand{\graytext}[1]{\textcolor{graytext}{#1}}
\newcommand{\redtext}[1]{\textcolor{redtext}{#1}}
%................................................................
%
%                              Other
%_______________________________________________________________
\if@wordlinks
    \RequirePackage[normalem]{ulem}
    \hypersetup{
    colorlinks=true,                % colored links
    urlcolor=wordhyperlink,         % blue color for url links
    linkcolor=black,
    anchorcolor=black,
    citecolor=black,
    filecolor=black
    }
    \makeatletter
    \DeclareUrlCommand\ULurl@@{%
      \def\UrlFont{\ttfamily\color{wordhyperlink}}%
      \def\UrlLeft{\uline\bgroup}%
      \def\UrlRight{\egroup}}
    \def\ULurl@#1{\hyper@linkurl{\ULurl@@{#1}}{#1}}
    \DeclareRobustCommand*\ULurl{\hyper@normalise\ULurl@}
    \makeatother
\else
    \newcommand{\ULurl}[1]{\url{#1}}
\fi
%
%
%                   Publications using bibLaTeX
%_______________________________________________________________
\if@nobib
% skip bibliography code
\else
% Load bib:
\RequirePackage[%
backend=biber,
style=apa,
sorting=nty
]{biblatex}
\fi
%................................................................
%
%                   At begin document
%_______________________________________________________________
\AtBeginDocument{%
    % some code?
}%
%................................................................
\endinput
%
%---------------------------------------------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                           Appendix 
%_______________________________________________________________
% ASCII-art created with ASCII art generator (name of style used: Georgia11) 
% Link to ASCII art generator: https://www.kammerl.de/ascii/AsciiSignature.php
%
%       File history:
%-------------------------------
% -- First version created in 2023
% -- 
%
%    Links and resources:
%-------------------------------
% -- Wordcount function:
%       https://www.overleaf.com/learn/how-to/Is_there_a_way_to_run_a_word_count_that_doesn't_include_LaTeX_commands%3F
% -- Times Font, difference from word pt 12:
%       https://tex.stackexchange.com/questions/270571/latex-12pt-is-different-from-actual-font-size-12-on-microsoft-word
% -- Fancyhdr documentation:
%       https://www.ctan.org/pkg/fancyhdr
%
%    TeX Stack Exchange users:
%-------------------------------
% Link to TeX.SE: https://tex.stackexchange.com/
% I would like to thank these TeX Stack Exchange (TeX.SE) users for helping me
% with parts of my LaTeX code:
% 
% -- 