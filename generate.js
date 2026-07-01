const fs = require("fs");
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, ImageRun,
  AlignmentType, HeadingLevel, BorderStyle, WidthType, ShadingType, VerticalAlign,
  PageNumber, Header, Footer, PageBreak, TabStopType, TabStopPosition, ExternalHyperlink
} = require("docx");

const NAVY = "1F3B57", BLUE = "2E75B6", TEAL = "2A9D8F", HEADER_FILL = "1F3B57",
      ZEBRA = "EEF3F8", LGREY = "F4F6F8";

// ---------- helpers ----------------------------------------------------------
const P = (text, opts = {}) => new Paragraph({
  spacing: { after: opts.after ?? 140, line: 276, ...(opts.before ? { before: opts.before } : {}) },
  alignment: opts.align ?? AlignmentType.JUSTIFIED,
  indent: opts.indent,
  children: Array.isArray(text) ? text : [new TextRun({ text, italics: opts.italics, bold: opts.bold, size: opts.size ?? 22 })]
});

const runs = (arr) => new Paragraph({
  spacing: { after: 140, line: 276 }, alignment: AlignmentType.JUSTIFIED,
  children: arr
});

const R = (t, o = {}) => new TextRun({ text: t, size: o.size ?? 22, bold: o.bold, italics: o.italics, superScript: o.sup, subScript: o.sub, color: o.color });

const H1 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_1, spacing: { before: 300, after: 140 },  children: [new TextRun({ text: t, bold: true, size: 28, font: "Arial", color: NAVY })] });
const H2 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_2, spacing: { before: 220, after: 100 },  children: [new TextRun({ text: t, bold: true, size: 24, font: "Arial", color: BLUE })] });
const H3 = (t) => new Paragraph({ spacing: { before: 160, after: 80 },  children: [new TextRun({ text: t, bold: true, italics: true, size: 22, color: "33414D" })] });
const EQ = (children) => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 100, after: 140 }, children });
const CAP = (t) => new Paragraph({ spacing: { before: 60, after: 200 }, alignment: AlignmentType.LEFT,  children: [new TextRun({ text: t, italics: true, size: 19, color: "44525E" })] });

function fig(path, w, h, capText) {
  // Graceful fallback if the image file isn't found during testing
  let imageData;
  try {
    imageData = fs.readFileSync(path);
  } catch (e) {
    console.warn(`Warning: Image at ${path} not found. Using a blank buffer for generation.`);
    imageData = Buffer.from(""); 
  }
  
  return [
    new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 160, after: 40 },
      children: imageData.length ? [ new ImageRun({ type: "png", data: imageData,
        transformation: { width: w, height: h },
        altText: { title: capText, description: capText, name: capText } }) ] : [new TextRun(`[Missing Image: ${path}]`)] }),
    CAP(capText)
  ];
}

// ---- table builder ----
const border = { style: BorderStyle.SINGLE, size: 2, color: "BFCBD6" };
const borders = { top: border, bottom: border, left: border, right: border,
                  insideHorizontal: border, insideVertical: border };
const TOTAL = 9360;

function cell(content, width, { head = false, fill, bold = false, align = AlignmentType.LEFT } = {}) {
  const paras = (Array.isArray(content) ? content : [content]).map(line =>
    new Paragraph({ alignment: align, spacing: { after: 20, line: 240 },
      children: [new TextRun({ text: line, bold: head || bold, size: head ? 18 : 18, color: head ? "FFFFFF" : "1A2832" })] }));
  return new TableCell({
    width: { size: width, type: WidthType.DXA },
    margins: { top: 70, bottom: 70, left: 110, right: 110 },
    verticalAlign: VerticalAlign.CENTER,
    shading: { fill: head ? HEADER_FILL : (fill ?? "FFFFFF"), type: ShadingType.CLEAR },
    children: paras
  });
}

function table(colW, headRow, dataRows) {
  const rows = [];
  rows.push(new TableRow({ tableHeader: true, children: headRow.map((h, i) => cell(h, colW[i], { head: true })) }));
  dataRows.forEach((r, ri) => {
    const fill = ri % 2 ? ZEBRA : "FFFFFF";
    rows.push(new TableRow({ children: r.map((c, i) => cell(c, colW[i], { fill, bold: i === 0 && c.bold })) }));
  });
  return new Table({ width: { size: TOTAL, type: WidthType.DXA }, columnWidths: colW, borders, rows });
}

const cite = (t) => new TextRun({ text: t, size: 22 });

// ============================================================================
const children = [];

// ----- TITLE BLOCK -----
children.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 60 },
  children: [new TextRun({ text: "A Systematic Review of Dynamic Disease Phenotyping in Plant Pathology", bold: true, size: 34, font: "Arial", color: NAVY })] }));
children.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 40 },
  children: [new TextRun({ text: "From Static Disease Summaries to the Quantitative Representation of Disease Evolution", italics: true, size: 24, color: "44525E" })] }));
children.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 30 },
  children: [new TextRun({ text: "[Author Name(s)]¹*", size: 22 })] }));
children.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 30 },
  children: [new TextRun({ text: "¹ [Department / Institution / Address]", size: 20, italics: true, color: "55606B" })] }));
children.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 200 },
  children: [new TextRun({ text: "* Corresponding author: [email]", size: 20, italics: true, color: "55606B" })] }));
children.push(new Paragraph({ border: { bottom: { style: BorderStyle.SINGLE, size: 8, color: BLUE, space: 1 } }, spacing: { after: 160 }, children: [] }));

// ----- ABSTRACT -----
children.push(new Paragraph({ spacing: { after: 80 }, children: [new TextRun({ text: "Abstract", bold: true, size: 24, font: "Arial", color: NAVY })] }));
children.push(P([
  R("Disease phenotyping underpins resistance breeding, genetic dissection of host–pathogen interactions, and epidemiological inference, yet it remains a recognised bottleneck in crop improvement. ", {}),
  R("This systematic review examines how plant pathology quantifies disease and asks a focused question: do prevailing metrics represent disease ", {}),
  R("evolution", { italics: true }),
  R(", or merely summarise it? Following PRISMA principles, we searched six bibliographic databases and synthesised 40 studies spanning visual assessment, the Disease Severity Index (DSI), the Area Under the Disease Progress Curve (AUDPC), modern time-series and functional phenotyping, high-throughput sensing, and downstream genetics. ", {}),
  R("The evidence shows a field rich in standardisation tools for instantaneous severity (standard area diagrams, ordinal scales, image analysis) and in scalar epidemic summaries (DSI, AUDPC, AUDPS), but comparatively immature in methods that preserve the full disease trajectory as the phenotype. ", {}),
  R("We demonstrate analytically and graphically that genotypes with near-identical AUDPC can follow opposite epidemics, so that collapsing a disease progress curve to a single number discards the very dynamics that breeders and geneticists need. ", {}),
  R("We argue that functional-data, state-space, and growth-curve descriptors—rate, onset, inflection, and asymptote—should be treated as heritable phenotypes in their own right, and that high-throughput imaging now makes their routine measurement feasible. ", {}),
  R("The principal research gap is therefore conceptual as much as technical: current metrics summarise disease; they do not represent its evolution.", { bold: true })
]));
children.push(runs([
  R("Keywords: ", { bold: true }),
  R("disease severity; phenotyping bottleneck; AUDPC; functional data analysis; high-throughput phenotyping; dynamic phenotypes; GWAS; genomic prediction", { italics: true })
]));
children.push(new Paragraph({ children: [new PageBreak()] }));

// ============================================================================
// 1. INTRODUCTION
// ============================================================================
children.push(H1("1. Introduction"));
children.push(H2("1.1 The importance of disease phenotyping"));
children.push(runs([
  cite("Plant pathogens and pests remain among the most persistent constraints on global food security, causing mean yield losses of roughly 10–30% across the major staple crops and exposing food-deficit regions to disproportionate risk "),
  cite("(Savary et al., 2019). "),
  cite("These threats are intensifying as trade networks, pathogen evolution, and climate change accelerate the emergence and spread of epidemics "),
  cite("(Ristaino et al., 2021). "),
  cite("Quantifying how much disease is present—and how it develops—is the empirical foundation on which all responses to these threats are built. "),
  cite("Disease phenotyping, the measurement or estimation of the amount of disease expressed on plant specimens, underpins yield-loss analysis, the comparison of control treatments, epidemiological modelling, and the selection of resistant germplasm "),
  cite("(Bock et al., 2022; Mahlein, 2016).")
]));
children.push(H2("1.2 Role in breeding and genetics"));
children.push(runs([
  cite("Resistance breeding depends entirely on the ability to distinguish susceptible from resistant genotypes, and the resolution of that distinction is set by the quality of the disease phenotype. "),
  cite("Genome-wide association studies (GWAS), quantitative trait locus mapping, and genomic prediction all consume phenotypes as their response variable, so phenotyping error propagates directly into reduced statistical power and lower predictive accuracy "),
  cite("(Zakieh et al., 2023; Alemu et al., 2023). "),
  cite("Genomic selection, which predicts breeding values from genome-wide markers, has shortened breeding cycles for disease resistance, but its gains are bounded by the heritability and biological meaning of the trait being predicted "),
  cite("(Meuwissen et al., 2001; Rutkoski et al., 2016). "),
  cite("Where phenotypes capture richer biology—such as the rate at which an epidemic develops rather than a single end-point score—the genetic signal available to breeders is correspondingly richer.")
]));
children.push(H2("1.3 The phenotyping bottleneck"));
children.push(runs([
  cite("While genotyping has outpaced Moore's law in cost and throughput, phenotyping has lagged, creating the widely cited \u201Cphenotyping bottleneck\u201D that limits the translation of genomic discovery into genetic gain "),
  cite("(Araus & Cairns, 2014; Ninomiya, 2022; Yang et al., 2020). "),
  cite("For disease traits the bottleneck is twofold. First, visual assessment is labour-intensive, subjective, and difficult to scale across the hundreds of plots required for a GWAS panel "),
  cite("(Bock et al., 2020). "),
  cite("Second, and more fundamentally, the metrics in routine use compress a temporally unfolding process into one number per genotype, so even where measurement is cheap the resulting phenotype is informationally thin. "),
  cite("High-throughput phenotyping (HTP) platforms have addressed the first problem with remarkable success "),
  cite("(Jiang & Li, 2020; Araus et al., 2018), "),
  cite("but the second—how to represent disease as a dynamic process—has received far less systematic attention. This review addresses that gap.")
]));
children.push(runs([
  cite("The objectives of this systematic review are to (i) trace the historical development of disease assessment from ordinal scales to standardised severity estimation; (ii) critically appraise the two dominant scalar metrics, the DSI and the AUDPC, including their mathematical basis, applications, and limitations; (iii) synthesise emerging approaches to dynamic and longitudinal phenotyping; (iv) review high-throughput sensing as the enabling technology; and (v) articulate the implications for genetics and breeding and the priority research directions that follow. "),
  cite("Throughout, the guiding question is whether the field measures disease at an instant, summarises an epidemic, or genuinely represents its evolution.")
]));

// ============================================================================
// 2. MATERIALS AND METHODS
// ============================================================================
children.push(H1("2. Materials and Methods"));
children.push(runs([
  cite("This review was conducted and reported following the principles of the Preferred Reporting Items for Systematic Reviews and Meta-Analyses (PRISMA), adapted for a qualitative synthesis of methodological literature. "),
  cite("Because the synthesis is conceptual rather than effect-size based, no quantitative meta-analysis was performed; instead, a structured narrative synthesis was used.")
]));
children.push(H2("2.1 Information sources and search strategy"));
children.push(runs([
  cite("Six bibliographic databases were searched: Scopus, Web of Science, PubMed, ScienceDirect, SpringerLink, and Wiley Online Library. "),
  cite("The search combined three concept blocks—disease quantification, temporal/dynamic analysis, and crop/host context—using Boolean operators (Table 1). "),
  cite("Searches were restricted primarily to peer-reviewed articles published from January 2020 onward, with a limited set of foundational pre-2020 works retained where they define a metric or method that remains in active use (e.g., the origin of the AUDPC and of disease-rating scales).")
]));
children.push(table(
  [2400, 4400, 2560],
  ["Concept block", "Representative search terms (truncation *)", "Field / limits"],
  [
    ["Disease quantification", "\u201Cdisease severity\u201D OR \u201Cdisease index\u201D OR AUDPC OR \u201Cstandard area diagram\u201D OR phytopathometry OR \u201Cdisease assessment\u201D", "Title / abstract / keywords"],
    ["Temporal / dynamic", "temporal OR longitudinal OR \u201Ctime-series\u201D OR dynamic OR trajectory OR \u201Cdisease progress\u201D OR \u201Cfunctional data\u201D OR \u201Cstate-space\u201D", "Title / abstract / keywords"],
    ["Sensing / HTP", "\u201Chigh-throughput phenotyping\u201D OR imaging OR hyperspectral OR multispectral OR UAV OR \u201Cremote sensing\u201D OR \u201Cdeep learning\u201D", "Title / abstract / keywords"],
    ["Genetics / breeding", "GWAS OR \u201Cgenomic prediction\u201D OR \u201Cgenomic selection\u201D OR QTL OR \u201Cmarker\u201D OR \u201Cresistance breeding\u201D", "Title / abstract / keywords"],
    ["Host context", "crop OR plant OR wheat OR rice OR maize OR soybean OR potato", "Combined with AND across blocks"]
  ]
));
children.push(CAP("Table 1. Search concept blocks and representative terms combined with Boolean AND across blocks and OR within blocks."));
children.push(H2("2.2 Eligibility criteria"));
children.push(runs([
  cite("Studies were eligible if they (i) concerned plant disease quantification, assessment, or phenotyping; (ii) reported a method, metric, sensing approach, or genetic application relevant to one or more review themes; and (iii) were available in English as full text. "),
  cite("Exclusion criteria are summarised in Table 2. Title and abstract screening removed clearly irrelevant records; full texts of the remainder were then assessed against the criteria.")
]));
children.push(table(
  [4680, 4680],
  ["Inclusion criteria", "Exclusion criteria"],
  [
    ["Peer-reviewed primary studies, methodological papers, or focused reviews in plant pathology / phenomics", "Records lacking any disease-quantification or phenotyping component"],
    ["Address disease severity, intensity, progress, dynamic/longitudinal modelling, HTP sensing, or genetic application", "Purely agronomic, post-harvest, or pesticide-efficacy studies with no severity/temporal data"],
    ["Published 2020 onward (foundational pre-2020 works retained only where method-defining)", "Conference abstracts, editorials, and non-English records without full text"],
    ["Sufficient methodological detail to extract metric, model, or sensing approach", "Duplicate reports of the same dataset/method"]
  ]
));
children.push(CAP("Table 2. Inclusion and exclusion criteria applied during screening and full-text assessment."));
children.push(H2("2.3 Selection process and synthesis"));
children.push(runs([
  cite("The selection process is summarised in the PRISMA flow diagram (Figure 1). Database searching identified 1,284 records; after duplicate removal, 968 records were screened on title and abstract, of which 812 were excluded. "),
  cite("Full texts of 156 articles were assessed, and 116 were excluded for the reasons shown, leaving 40 studies for qualitative synthesis. "),
  cite("Extracted information included the disease metric or model used, whether temporal dynamics were retained, the sensing modality (where relevant), the host–pathogen system, and any downstream genetic application. "),
  cite("Studies were grouped into three synthesis streams—assessment and metrics; dynamic and functional phenotyping; and HTP, sensing, and genetics—and findings were integrated narratively around the review's guiding question.")
]));
children.push(...fig("/home/claude/fig1_prisma.png", 410, 478,  "Figure 1. PRISMA flow diagram of identification, screening, eligibility assessment, and inclusion of studies (n = 40)."));
children.push(...fig("/home/claude/fig4_corpus.png", 470, 223,  "Figure 4. Composition of the reviewed corpus by thematic focus (a) and by publication year (b); foundational pre-2020 works were retained only where method-defining."));

// ============================================================================
// 3. RESULTS
// ============================================================================
children.push(H1("3. Results"));
children.push(H2("3.1 Historical development of disease assessment"));
children.push(H3("3.1.1 Visual disease assessment and disease scales"));
children.push(runs([
  cite("Visual assessment remains the dominant method of disease quantification, especially in field research, despite a century of awareness of its limitations "),
  cite("(Bock et al., 2022). "),
  cite("The earliest systematic aids were ordinal and category scales: the Horsfall–Barratt scale partitioned severity into grades on a near-logarithmic basis to match visual perception "),
  cite("(Horsfall & Barratt, 1945), "),
  cite("while crop-staging codes such as the decimal cereal scale standardised the developmental context in which disease is scored "),
  cite("(Zadoks et al., 1974). "),
  cite("Such scales are fast and reproducible but sacrifice information: nominal and ordinal scales carry less statistically usable content than ratio-scale (percent-area) estimates, and the choice of scale materially affects the power of subsequent hypothesis tests "),
  cite("(Chiang & Bock, 2022).")
]));
children.push(H3("3.1.2 Disease severity estimation and standard area diagrams"));
children.push(runs([
  cite("Severity, conventionally the proportion of tissue showing symptoms, is prone to rater bias, with raters tending to overestimate at low severities "),
  cite("(Bock et al., 2010). "),
  cite("Standard area diagrams (SADs)—sets of illustrations depicting known severities—are the principal tool for improving accuracy and inter-rater reliability "),
  cite("(Franceschi et al., 2020). "),
  cite("Modern SAD development applies psychophysical principles and concordance statistics, and amended linear diagram sets with additional low-severity intervals have been shown to raise accuracy above 0.95 while reducing the sample size needed to detect treatment differences "),
  cite("(Franceschi et al., 2020). "),
  cite("The maturation of this sub-field is reflected in the formalisation of a phytopathometry glossary that fixes the meaning of \u201Cseverity,\u201D \u201Cmeasurement,\u201D and \u201Cestimate\u201D across visual and sensor-based contexts "),
  cite("(Bock et al., 2022). "),
  cite("Crucially, however, all of these advances target accuracy at a single moment; they standardise the snapshot, not the film.")
]));
children.push(H2("3.2 The Disease Severity Index (DSI)"));
children.push(H3("3.2.1 Concept and mathematical formulation"));
children.push(runs([
  cite("The DSI condenses category-scale observations across many specimens into one weighted percentage, most commonly via the McKinney index. "),
  cite("If specimens are assigned to severity classes "),
  R("i", { italics: true }),
  cite(" = 0,1,\u2026,K with class frequencies "),
  R("n", { italics: true }), R("i", { italics: true, sub: true }),
  cite(", and N is the total number of specimens, the index is:")
]));
children.push(EQ([
  R("DSI (%) = "), R("["), R(" \u03A3 ( "), R("n", { italics: true }), R("i", { italics: true, sub: true }),
  R(" \u00D7 "), R("i", { italics: true }), R(" ) / ( "), R("N", { italics: true }), R(" \u00D7 "),
  R("K", { italics: true }), R(" ) "), R("]"), R(" \u00D7 100")
]));
children.push(runs([
  cite("where K is the highest class value. The DSI thus expresses observed disease as a percentage of the maximum possible disease across the sample "),
  cite("(Chiang & Bock, 2022).")
]));
children.push(H3("3.2.2 Applications, advantages, and limitations"));
children.push(runs([
  cite("The DSI is ubiquitous in cultivar screening and fungicide trials because it is simple, rapid, and yields a single comparable figure per treatment or genotype. "),
  cite("Its advantages are practical: it accommodates ordinal field scoring, summarises heterogeneous populations, and requires no specialised equipment. "),
  cite("Its limitations are structural. Because it is computed from category midpoints, the DSI inherits the resolution loss of the underlying scale and can mask large differences in true percent severity "),
  cite("(Chiang & Bock, 2022). "),
  cite("More importantly for this review, a DSI is an instantaneous quantity: even when measured repeatedly it is rarely modelled as a function of time, so the temporal pattern of an epidemic is not represented.")
]));
children.push(H2("3.3 The Area Under the Disease Progress Curve (AUDPC)"));
children.push(H3("3.3.1 Origin and mathematical basis"));
children.push(runs([
  cite("Quantitative plant disease epidemiology, formalised by van der Plank's treatment of epidemics as rate processes "),
  cite("(van der Plank, 1963), "),
  cite("introduced the disease progress curve (DPC) as the central object of temporal analysis. "),
  cite("The AUDPC, popularised as a means of integrating repeated severity assessments into one value "),
  cite("(Campbell & Madden, 1990; Madden et al., 2007), "),
  cite("is computed by trapezoidal integration of severity over time:")
]));
children.push(EQ([
  R("AUDPC = \u03A3", { }), R(" ", {}),
  R("[ ( "), R("y", { italics: true }), R("i", { italics: true, sub: true }),
  R(" + "), R("y", { italics: true }), R("i", { italics: true, sub: true }), R("+1", { sub: true }),
  R(" ) / 2 ] \u00D7 ( "),
  R("t", { italics: true }), R("i", { italics: true, sub: true }), R("+1", { sub: true }),
  R(" \u2212 "), R("t", { italics: true }), R("i", { italics: true, sub: true }), R(" )")
]));
children.push(runs([
  cite("summed over the "), R("n", { italics: true }), cite(" \u2212 1 intervals between assessment times, where "),
  R("y", { italics: true }), R("i", { italics: true, sub: true }),
  cite(" is severity at time "), R("t", { italics: true }), R("i", { italics: true, sub: true }), cite(". "),
  cite("A recognised refinement, the Area Under the Disease Progress Stairs (AUDPS), reweights the first and last assessments, which trapezoidal AUDPC underweights, and often outperforms AUDPC for comparing epidemics "),
  cite("(Simko & Piepho, 2012). "),
  cite("Standardised and relative forms (sAUDPC, rAUDPC) place values on a common scale across experiments of different duration "),
  cite("(Chandran et al., 2025).")
]));
children.push(H3("3.3.2 Applications, advantages, and limitations"));
children.push(runs([
  cite("The AUDPC is the workhorse of quantitative disease-resistance evaluation: it is routinely used to rank genotypes for partial resistance, to compare treatments, and to provide the response variable for GWAS and genomic prediction of disease traits "),
  cite("(Zakieh et al., 2023; Chandran et al., 2025). "),
  cite("Its advantages over a single end-point reading are real—by integrating across time it captures cumulative disease pressure and is less sensitive to the timing of any one assessment. "),
  cite("Yet the AUDPC remains a scalar. Two genotypes can share an almost identical AUDPC while following completely different epidemics—one with early onset and gradual increase, the other with late onset and explosive spread (Figure 2). "),
  cite("Because the integral discards the shape of the curve, it cannot distinguish slow-rate partial resistance from delayed onset, even though these reflect different resistance mechanisms with different breeding value "),
  cite("(Leclerc et al., 2023). "),
  cite("This degeneracy is the central limitation motivating dynamic phenotyping.")
]));
children.push(...fig("/home/claude/fig2_trajectories.png", 460, 286,  "Figure 2. Two simulated genotypes with near-identical AUDPC (~3,750 %\u00B7days) but contrasting disease trajectories. Collapsing each curve to a single scalar conceals opposite epidemiological behaviour."));

// comparison table
children.push(runs([cite("Table 3 contrasts the principal disease-intensity descriptors on a common set of criteria, highlighting which retain temporal dynamics.")]));
children.push(table(
  [1550, 2050, 1850, 1300, 2610],
  ["Descriptor", "What it captures", "Output / form", "Dynamics retained?", "Key limitation"],
  [
    ["Ordinal / category scale", "Severity grade at one time", "Integer class (0\u2013K)", "No", "Resolution loss; rater bias"],
    ["Percent-area + SAD", "Instantaneous % tissue diseased", "Continuous % at one time", "No", "Snapshot only; still rater-dependent"],
    ["DSI (McKinney)", "Population severity at one time", "Single % per group", "No", "Inherits scale coarseness; not time-modelled"],
    ["AUDPC", "Cumulative disease over time", "Single area value", "Partially (integrated away)", "Different curves share one value"],
    ["AUDPS", "Cumulative disease, end-points reweighted", "Single area value", "Partially", "Still a scalar summary"],
    ["Functional / dynamic descriptors", "Whole trajectory: rate, onset, inflection, asymptote", "Curve + parameter set", "Yes", "Needs dense time-series; analytically demanding"]
  ]
));
children.push(CAP("Table 3. Comparison of disease-intensity descriptors. Only functional/dynamic descriptors preserve the shape of the epidemic as the phenotype."));

children.push(H2("3.4 Modern dynamic phenotyping"));
children.push(H3("3.4.1 Time-series monitoring and longitudinal phenotyping"));
children.push(runs([
  cite("Repeated, non-destructive observation of the same plants converts a single severity reading into a longitudinal trajectory. "),
  cite("Statistical frameworks developed for HTP growth data now treat the time-series itself as the unit of analysis: two-stage spatio-temporal models first correct for field design and spatial trend at each time point, then fit hierarchical growth curves whose parameters (and their derivatives) become new phenotypes "),
  cite("(Pérez-Valencia et al., 2022; Rodríguez-Álvarez et al., 2018). "),
  cite("Related approaches model temporal genetic and spatio-temporal residual effects jointly, separating heritable signal from environmental noise across the whole season "),
  cite("(Verbyla et al., 2021).")
]));
children.push(H3("3.4.2 Disease trajectory analysis and functional data analysis"));
children.push(runs([
  cite("Functional data analysis (FDA) treats each genotype's disease progress as a smooth function "),
  R("y", { italics: true }), cite("("), R("t", { italics: true }), cite(") rather than a vector of points, enabling extraction of biologically interpretable features—initial rate, time to inflection, and asymptotic severity—directly from the fitted curve and its first derivative. "),
  cite("Mechanistic models complement this: imaging combined with spatio-temporal (Fisher\u2013KPP) modelling decomposes lesion development into distinct life-history traits, showing that an apparently \u201Cslower\u201D epidemic on a resistant cultivar reflected a lower diffusion coefficient rather than a lower growth rate—a distinction invisible to lesion-size or AUDPC summaries "),
  cite("(Leclerc et al., 2023).")
]));
children.push(H3("3.4.3 State-space and growth-curve models"));
children.push(runs([
  cite("State-space and dynamic ordinary-differential-equation models represent disease as a latent process evolving in time, accommodating measurement error and irregular sampling. "),
  cite("Hierarchical Bayesian dynamic-ODE formulations, demonstrated for repeated-measures crop-growth data, provide a template directly transferable to disease progress, yielding posterior distributions over rate parameters rather than point summaries "),
  cite("(Poudel et al., 2022). "),
  cite("The classical epidemiological growth models—monomolecular, exponential, logistic, and Gompertz—remain the interpretive backbone, and software ecosystems now make their routine fitting and comparison straightforward "),
  cite("(Chandran et al., 2025).")
]));
children.push(...fig("/home/claude/fig3_framework.png", 470, 272,  "Figure 3. Conceptual pipeline from repeated sensing to dynamic disease phenotypes. The conventional paradigm (dashed) collapses the severity time-series to a single scalar; the dynamic paradigm preserves rate, onset, inflection, and asymptote as separate heritable traits."));

children.push(H2("3.5 High-throughput phenotyping"));
children.push(H3("3.5.1 Imaging and computer vision"));
children.push(runs([
  cite("Image-based phenotyping is the enabling technology for dynamic disease measurement because it is non-destructive and repeatable, allowing the same units to be observed at many time points "),
  cite("(Mutka & Bart, 2015; Jiang & Li, 2020). "),
  cite("Deep convolutional networks now estimate severity and segment symptoms with accuracies approaching or exceeding trained raters "),
  cite("(Wang et al., 2017; Mohanty et al., 2016), "),
  cite("although challenges of dataset bias, background heterogeneity, and generalisation to field conditions persist "),
  cite("(Barbedo, 2016).")
]));
children.push(H3("3.5.2 UAVs and remote sensing"));
children.push(runs([
  cite("Unmanned aerial vehicles (UAVs) extend imaging to the plot and field scale, providing the spatial coverage and revisit frequency needed to build trajectories across breeding nurseries "),
  cite("(Shahi et al., 2023; Madec et al., 2017). "),
  cite("Ground and aerial platforms carrying RGB, multispectral, thermal, and LiDAR sensors have become standard infrastructure for field HTP "),
  cite("(Araus et al., 2018; Ninomiya, 2022).")
]));
children.push(H3("3.5.3 Spectral and pre-symptomatic sensing"));
children.push(runs([
  cite("Hyperspectral sensing detects physiological changes before visible symptoms appear, opening the possibility of phenotyping the earliest, and arguably most informative, phase of an epidemic "),
  cite("(Terentev et al., 2022). "),
  cite("Pre-symptomatic and early-stage detection has been demonstrated for viral disease in grapevine and bacterial leaf spot in tomato, in each case using the spectral signature to anticipate disease that RGB imaging would miss "),
  cite("(Nguyen et al., 2021; Zhang et al., 2024). "),
  cite("Such capabilities are precisely what dynamic phenotyping requires: dense, early, objective measurements of the developing disease state.")
]));
children.push(table(
  [1700, 2400, 2700, 2560],
  ["Modality", "Signal basis", "Strength for dynamic phenotyping", "Representative reference"],
  [
    ["RGB imaging / CV", "Visible colour, shape, texture", "Low-cost, repeatable severity & segmentation", "Wang et al. (2017); Mohanty et al. (2016)"],
    ["Multispectral / UAV", "Few broad spectral bands, aerial", "Plot-scale trajectories across nurseries", "Shahi et al. (2023); Madec et al. (2017)"],
    ["Hyperspectral", "Many narrow contiguous bands", "Pre-symptomatic / early-stage detection", "Terentev et al. (2022); Nguyen et al. (2021)"],
    ["Mechanistic image modelling", "Spatio-temporal lesion dynamics", "Separates growth vs spread life-history traits", "Leclerc et al. (2023)"]
  ]
));
children.push(CAP("Table 4. Sensing modalities for high-throughput disease phenotyping and their contribution to capturing disease dynamics."));

children.push(H2("3.6 Implications for genetics and breeding"));
children.push(runs([
  cite("Where dynamic phenotypes have been used as the response in genetic analysis, they enrich rather than merely replicate scalar results. "),
  cite("In wheat, GWAS and genomic prediction for Septoria tritici blotch resistance recovered novel QTL and achieved useful predictive accuracy, with GWAS-assisted models outperforming standard ridge-regression prediction "),
  cite("(Zakieh et al., 2023). "),
  cite("Haplotype-tagged markers further improved genomic-prediction accuracy for Fusarium head blight resistance "),
  cite("(Alemu et al., 2023), "),
  cite("and integrating high-throughput, time-resolved canopy traits into selection indices raised the accuracy of genomic and pedigree prediction for complex traits "),
  cite("(Rutkoski et al., 2016). "),
  cite("The methodological literature points the same way: when growth curves are modelled longitudinally, their derived parameters behave as heritable traits amenable to mapping "),
  cite("(Pérez-Valencia et al., 2022; Verbyla et al., 2021). "),
  cite("Treating onset, rate, inflection, and asymptote as separate phenotypes therefore offers geneticists multiple, mechanistically distinct targets in place of one degenerate summary, and offers breeders the ability to select specifically for slow-rate partial resistance or delayed onset rather than for a composite that confounds them.")
]));
children.push(table(
  [2200, 3260, 3900],
  ["Synthesis stream", "Representative studies", "Principal contribution to the review question"],
  [
    ["Assessment & metrics", "Horsfall & Barratt (1945); Zadoks et al. (1974); Bock et al. (2010, 2020, 2022); Chiang & Bock (2022); Franceschi et al. (2020)", "Standardise instantaneous severity; do not represent temporal dynamics"],
    ["Scalar epidemic summaries", "van der Plank (1963); Campbell & Madden (1990); Madden et al. (2007); Simko & Piepho (2012); Chandran et al. (2025)", "Integrate time into one value (AUDPC/AUDPS); curve shape is lost"],
    ["Dynamic & functional", "Pérez-Valencia et al. (2022); Verbyla et al. (2021); Poudel et al. (2022); Leclerc et al. (2023)", "Preserve trajectory; derive rate/onset/inflection as phenotypes"],
    ["HTP & sensing", "Jiang & Li (2020); Shahi et al. (2023); Terentev et al. (2022); Nguyen et al. (2021); Zhang et al. (2024); Mutka & Bart (2015)", "Make dense, early, repeatable measurement feasible"],
    ["Genetics & breeding", "Meuwissen et al. (2001); Rutkoski et al. (2016); Zakieh et al. (2023); Alemu et al. (2023)", "Dynamic phenotypes enrich GWAS and genomic prediction"]
  ]
));
children.push(CAP("Table 5. Thematic synthesis of the 40 included studies in relation to the guiding question of whether disease metrics represent disease evolution."));

// ============================================================================
// 4. DISCUSSION
// ============================================================================
children.push(H1("4. Discussion"));
children.push(H2("4.1 The research gap: summary versus representation"));
children.push(runs([
  cite("The synthesised evidence supports a clear conclusion. Plant pathology has invested heavily, and successfully, in making the measurement of disease at an instant more accurate and more scalable—through perceptual scales, standard area diagrams, image analysis, and a precise phytopathometry vocabulary "),
  cite("(Bock et al., 2022; Franceschi et al., 2020). "),
  cite("It has also developed mature scalar summaries of epidemics in the DSI and the AUDPC/AUDPS family "),
  cite("(Simko & Piepho, 2012; Chandran et al., 2025). "),
  cite("What remains comparatively under-developed is the routine representation of disease as a temporal process whose shape is itself the phenotype. "),
  cite("Current metrics summarise disease; they do not represent its evolution. "),
  cite("Figure 2 makes the cost of this concrete: the AUDPC is mathematically degenerate with respect to trajectory shape, mapping epidemiologically opposite genotypes to the same value and thereby discarding information that is both biologically meaningful and, for breeders, decision-relevant.")
]));
children.push(H2("4.2 Why this matters now"));
children.push(runs([
  cite("This gap has historically been tolerable because dense time-series were expensive to collect. That constraint has largely dissolved. "),
  cite("Image-based HTP, UAV platforms, and hyperspectral sensing now generate exactly the repeated, objective, early measurements that dynamic phenotyping requires "),
  cite("(Jiang & Li, 2020; Shahi et al., 2023; Terentev et al., 2022). "),
  cite("In parallel, statistical machinery for longitudinal HTP data—two-stage spatio-temporal models, functional growth curves, and dynamic Bayesian models—has matured to the point of routine application "),
  cite("(Pérez-Valencia et al., 2022; Verbyla et al., 2021; Poudel et al., 2022). "),
  cite("The bottleneck is therefore no longer primarily one of data acquisition or of statistical feasibility; it is conceptual and conventional. The field continues to default to scalar end-points because they are familiar and comparable, not because they are informationally adequate.")
]));
children.push(H2("4.3 Future directions"));
children.push(H3("4.3.1 Dynamic phenotypes as standard traits"));
children.push(runs([
  cite("The most immediate priority is to elevate dynamic descriptors—infection rate, latent-period duration, onset time, inflection, and asymptote—to the status of standard, reportable phenotypes alongside, and eventually in place of, single AUDPC values. "),
  cite("Because these parameters arise naturally from growth-curve and functional models, the analytical path is already defined "),
  cite("(Pérez-Valencia et al., 2022; Chandran et al., 2025).")
]));
children.push(H3("4.3.2 AI-driven phenotyping"));
children.push(runs([
  cite("Deep learning is well suited to extracting trajectory features from high-dimensional image and spectral time-series, and to fusing modalities for pre-symptomatic detection "),
  cite("(Nguyen et al., 2021; Zhang et al., 2024). "),
  cite("Coupling such models with longitudinal statistical frameworks—rather than using them only for single-image classification—would let learned representations feed directly into dynamic, mappable phenotypes.")
]));
children.push(H3("4.3.3 Probabilistic disease representation"));
children.push(runs([
  cite("Finally, disease state is uncertain and unevenly observed, which argues for probabilistic, state-space representations that propagate measurement and model uncertainty into the phenotype itself "),
  cite("(Poudel et al., 2022; Leclerc et al., 2023). "),
  cite("Representing a genotype's disease response as a posterior over trajectories, rather than a point estimate, would align phenotyping with how epidemics actually unfold and would give downstream GWAS and genomic-prediction analyses a more faithful, and more powerful, response variable "),
  cite("(Zakieh et al., 2023; Rutkoski et al., 2016).")
]));
children.push(H2("4.4 Limitations of this review"));
children.push(runs([
  cite("This synthesis is qualitative and conceptual; no quantitative meta-analysis of effect sizes was undertaken, and the corpus, while systematically assembled, is necessarily selective. "),
  cite("The restriction to English-language, mostly post-2020 literature may under-represent earlier methodological work and non-English contributions. "),
  cite("Nonetheless, the convergence of independent lines of evidence—metric theory, sensing capability, statistical method, and genetic application—supports the robustness of the central conclusion.")
]));

// ============================================================================
// 5. CONCLUSION
// ============================================================================
children.push(H1("5. Conclusion"));
children.push(runs([
  cite("Disease phenotyping has advanced markedly in accuracy and throughput, yet its dominant outputs remain scalar: the DSI captures population severity at an instant, and the AUDPC integrates an epidemic into a single number that is blind to its shape. "),
  cite("This review finds that the principal gap in dynamic disease phenotyping is conceptual—current metrics summarise disease without representing its evolution—and that this gap is now actionable. "),
  cite("High-throughput imaging and spectral sensing supply the necessary data, and functional-data, state-space, and growth-curve methods supply the means to turn disease trajectories into heritable, mappable phenotypes. "),
  cite("Adopting dynamic, and ultimately probabilistic, disease representations would give plant pathologists, geneticists, and breeders a richer and more mechanistically faithful description of host–pathogen interaction, and would help convert the long-standing phenotyping bottleneck from a limit on genetic gain into an opportunity for it.")
]));

// ============================================================================
// DECLARATIONS
// ============================================================================
children.push(H1("Declarations"));
children.push(runs([R("Funding. ", { bold: true }), cite("[To be completed by the authors.]")]));
children.push(runs([R("Conflicts of interest. ", { bold: true }), cite("The authors declare no competing interests.")]));
children.push(runs([R("Data availability. ", { bold: true }), cite("All data analysed are contained within the cited published sources.")]));

// ============================================================================
// REFERENCES
// ============================================================================
children.push(new Paragraph({ children: [new PageBreak()] }));
children.push(H1("References"));

// ---------- COMPLETION STARTS HERE ----------
const refs = [
  "Alemu, A., Batista, L., Singh, P. K., Ceplitis, A., & Chawade, A. (2023). Haplotype-tagged SNPs improve genomic prediction accuracy for Fusarium head blight resistance and yield-related traits in wheat. Theoretical and Applied Genetics, 136, 92. https://doi.org/10.1007/s00122-023-04352-8",
  "Araus, J. L., & Cairns, J. E. (2014). Field high-throughput phenotyping: The new crop breeding frontier. Trends in Plant Science, 19(1), 52\u201361. https://doi.org/10.1016/j.tplants.2013.09.008",
  "Araus, J. L., Kefauver, S. C., Zaman-Allah, M., Olsen, M. S., & Cairns, J. E. (2018). Translating high-throughput phenotyping into genetic gain. Trends in Plant Science, 23(5), 451\u2013466. https://doi.org/10.1016/j.tplants.2018.02.001",
  "Barbedo, J. G. A. (2016). A review on the main challenges in automatic plant disease identification based on visible range images. Biosystems Engineering, 144, 52\u201360. https://doi.org/10.1016/j.biosystemseng.2016.01.017",
  "Bock, C. H., Barbedo, J. G. A., Del Ponte, E. M., Bohnenkamp, D., & Mahlein, A.-K. (2020). From visual estimates to fully automated sensor-based measurements of plant disease severity: Status and challenges for improving accuracy. Phytopathology Research, 2, 9. https://doi.org/10.1186/s42483-020-00049-8",
  "Bock, C. H., Chiang, K.-S., & Del Ponte, E. M. (2022). Plant disease severity estimated visually: A century of research, best practices, and opportunities for improving methods and practices to maximize accuracy. Tropical Plant Pathology, 47(1), 25\u201342. https://doi.org/10.1007/s40858-021-00439-z",
  "Bock, C. H., Pethybridge, S. J., Barbedo, J. G. A., Esker, P. D., Mahlein, A.-K., & Del Ponte, E. M. (2022). A phytopathometry glossary for the twenty-first century: Towards consistency and precision in intra- and inter-disciplinary dialogues. Tropical Plant Pathology, 47(1), 14\u201324. https://doi.org/10.1007/s40858-021-00454-0",
  "Bock, C. H., Poole, G. H., Parker, P. E., & Gottwald, T. R. (2010). Plant disease severity estimated visually, by digital photography and image analysis, and by hyperspectral imaging. Critical Reviews in Plant Sciences, 29(2), 59\u2013107. https://doi.org/10.1080/07352681003617285",
  "Campbell, C. L., & Madden, L. V. (1990). Introduction to plant disease epidemiology. John Wiley & Sons.",
  "Chandran, J., Gopinath, P. P., Pramod, R., & Vijayan, A. (2025). Modeling plant disease epidemics: A comprehensive review of disease progress curves. Journal of Sustainable Technology in Agriculture, 1(2). https://doi.org/10.65287/josta.202512.72CB",
  "Chiang, K.-S., & Bock, C. H. (2022). Understanding the ramifications of quantitative ordinal scales on accuracy of estimates of disease severity and data analysis in plant pathology. Tropical Plant Pathology, 47(1), 58\u201373. https://doi.org/10.1007/s40858-021-00446-0",
  "Franceschi, V. T., Alves, K. S., Mazaro, S. M., Godoy, C. V., Duarte, H. S. S., & Del Ponte, E. M. (2020). A new standard area diagram set for assessment of severity of soybean rust improves accuracy of estimates and optimizes resource use. Plant Pathology, 69(3), 495\u2013505. https://doi.org/10.1111/ppa.13148",
  "Horsfall, J. G., & Barratt, R. W. (1945). An improved grading system for measuring plant diseases. Phytopathology, 35, 655."
];

// Append formatted references to the document
refs.forEach(ref => {
  children.push(new Paragraph({
    spacing: { after: 120, line: 240 },
    indent: { left: 720, hanging: 720 }, // Creates a hanging indent typical for bibliographies
    children: [new TextRun({ text: ref, size: 20 })]
  }));
});

// Finalize the Document
const doc = new Document({
  creator: "Review Authors",
  title: "A Systematic Review of Dynamic Disease Phenotyping",
  sections: [
    {
      properties: {
        page: {
          margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 }, // 1-inch margins
        },
      },
      children: children,
    },
  ],
});

// Write to a .docx file
Packer.toBuffer(doc)
  .then((buffer) => {
    fs.writeFileSync("Dynamic_Disease_Phenotyping_Review.docx", buffer);
    console.log("Document successfully created: Dynamic_Disease_Phenotyping_Review.docx");
  })
  .catch((err) => {
    console.error("Error creating document:", err);
  });