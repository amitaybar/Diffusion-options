The code is self contained and was tested using Matlab 2018b.

The main file is "Main.m". It calls all the other functions and scripts.

Inside Main.m, in lines 14-17 there are flags controling the created domain. Please note that only one of the flags should have a logical '1' value at a time. The parameter DiffusionTime in line 23 sets the t scale parameter of the algorithm.

Figures 1 and 2 in the paper are created by running Main.m with different domain flags and a scale parameter t.

Figure 3 in the paper is created by running Main.m using the flag IsStochasticDomainFlag = 1 (line 19).

The data in Table 1 was created by running Main.m using the flag IsExplorStepsStatesFlag (line 21). Choosing between diffusion options, eigenoptinos and randomo walk is done by setting the flags in the file DiffutionTimeCalcWithOptions.m, lines 13 and 14. The flag DiffusionOptionsFlag contrlos which set of options is used, and the flag OnlyPrimitiveAction enforces only primitive actions (thus, should be disables when options are used). The results (for the corresponding aforementioned flags) appear in the console at the end of the run.  
  
