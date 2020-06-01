inputFile = "RK5_20200104_SHSY5Y_ctrlctrl_0_03_Alexa488_06.tif";
pathName = "ctrlctrl/";
outputFile = "RK5_20200104_SHSY5Y_ctrlctrl_0_03_Alexa488_06

open("D:/Master Thesis/ Kooperation - automatisierte Neuritenmessung/Mikroskopaufnahmen/RK5_20200104_SHSY5Y_2_20180719_P15_20200128/" + pathName + inputFile );
selectWindow(inputFile);
run("Brightness/Contrast...");
selectWindow("B&C");
getMinAndMax(min, max);
setMinAndMax(min, (max/2));
run("Apply LUT");
run("Close");
run("Brightness/Contrast...");
selectWindow("B&C");
getMinAndMax(min, max);
setMinAndMax(min, (max/2));
run("Apply LUT");
run("Close");
selectWindow(inputFile);
run("8-bit");
run("Auto Local Threshold...", "method=Phansalkar radius=35 parameter_1=0 parameter_2=0 white");
saveAs("PNG", "C:/Users/nkiwa/OneDrive/Desktop/MitMakroVorverarbeitet2/" + outputFile);
close();
