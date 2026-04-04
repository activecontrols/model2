dictObj = Simulink.data.dictionary.open('Variables.sldd');
dataSect = getSection(dictObj, 'Design Data');
% addEntry(dataSect, 'TESTTT', 5);
constantsASTRA = getEntry(dataSect, 'constantsASTRA');