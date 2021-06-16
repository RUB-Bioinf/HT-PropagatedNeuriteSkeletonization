inputDir = getArgument();
print(inputDir);
processFolder(inputDir);



function processAlexaFile(input){
	print("3");

	open(input);
	subStr = split(input, ".");

	/*
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	*/
	
	run("8-bit");
	run("Auto Local Threshold", "method=Phansalkar radius=10 parameter_1=0 parameter_2=0 white");
	saveAs("PNG", ".."+ subStr[0]);
	close();
}

function processDapiFile(input){
	print("2");
	
	open(input);
	subStr = split(input, ".");

	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	run("8-bit");
	saveAs("PNG", ".."+ subStr[0]);
	close();
}

function processFolder(input){
	print("1");
	list = getFileList(input);
	for(i = 0; i < list.length; i++){
		if(File.isDirectory(input + list[i]))
		{
			print(input + list[i]);
			print("8");
			processFolder(input + list[i]);
		}
		if (endsWith(list[i], ".tif"))
		{
			if (matches(list[i], ".*Alexa.*")){
				print(input + list[i]);
				print("9");
				processAlexaFile(input + list[i]);
			}
			if (matches(list[i], ".*DAPI.*")){
				print(input + list[i]);
				processDapiFile(input + list[i]);
			}
		}
	}
	print("ImageJ Vorverarbeitung fertig");
}

