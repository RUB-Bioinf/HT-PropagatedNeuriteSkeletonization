inputDir = getArgument();
processFolder(inputDir);
print("fertig");


function processAlexaFile(input){
	open(input);
	subStr = split(input, ".");

<<<<<<< HEAD
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
=======
>>>>>>> dc860fc295fedf057e8386a4441ff30731ba9753
	run("8-bit");
	run("Auto Local Threshold", "method=Phansalkar radius=35 parameter_1=0 parameter_2=0 white");
	saveAs("PNG", ".."+ subStr[0]);
	close();
}

function processDapiFile(input){
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
	list = getFileList(input);
	for(i = 0; i < list.length; i++){
		if(File.isDirectory(input + list[i]))
		{
			processFolder(input + list[i]);
		}
		if (endsWith(list[i], ".tif"))
		{
			if (matches(list[i], ".*Alexa.*")){
				processAlexaFile(input + list[i]);
			}
			if (matches(list[i], ".*DAPI.*")){
				processDapiFile(input + list[i]);
			}
		}
	}
}

