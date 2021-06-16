inputDir = getArgument();
print("Starting ImageJ Preprocessing");
processFolder(inputDir);
print("Finished ImageJ Preprocessing");


function processAlexaFile(input){
	print("ImageJ Processing an unknwon Alexa File");
	open(input);
	subStr = split(input, ".");
	
	run("8-bit");
	run("Auto Local Threshold", "method=Phansalkar radius=35 parameter_1=0 parameter_2=0 white");
	print("ImageJ Finished Processing an unknwon Alexa File. Saving.");
	saveAs("PNG", ".."+ subStr[0]);
	close();
}

function processDapiFile(input){
	print("ImageJ Processing an unknwon Dapi File");
	open(input);
	subStr = split(input, ".");
	
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	getMinAndMax(min, max);
	setMinAndMax(min, (max/2));
	run("Apply LUT");
	run("8-bit");
	print("ImageJ Finished Processing an unknwon Dapi File. Saving.");
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

