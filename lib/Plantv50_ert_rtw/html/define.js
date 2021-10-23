function CodeDefine() { 
this.def = new Array();
this.def["ert_main.cpp:rtObj"] = {file: "ert_main_cpp.html",line:26,type:"var"};
this.def["rt_OneStep"] = {file: "ert_main_cpp.html",line:40,type:"fcn"};
this.def["main"] = {file: "ert_main_cpp.html",line:76,type:"fcn"};
this.def["BigEndianIEEEDouble"] = {file: "Plantv50_cpp.html",line:76,type:"type"};
this.def["LittleEndianIEEEDouble"] = {file: "Plantv50_cpp.html",line:83,type:"type"};
this.def["IEEESingle"] = {file: "Plantv50_cpp.html",line:90,type:"type"};
this.def["rtInf"] = {file: "Plantv50_cpp.html",line:94,type:"var"};
this.def["rtMinusInf"] = {file: "Plantv50_cpp.html",line:95,type:"var"};
this.def["rtNaN"] = {file: "Plantv50_cpp.html",line:96,type:"var"};
this.def["rtInfF"] = {file: "Plantv50_cpp.html",line:97,type:"var"};
this.def["rtMinusInfF"] = {file: "Plantv50_cpp.html",line:98,type:"var"};
this.def["rtNaNF"] = {file: "Plantv50_cpp.html",line:99,type:"var"};
this.def["rtGetNaN"] = {file: "Plantv50_cpp.html",line:114,type:"fcn"};
this.def["rtGetNaNF"] = {file: "Plantv50_cpp.html",line:138,type:"fcn"};
this.def["rt_InitInfAndNaN"] = {file: "Plantv50_cpp.html",line:152,type:"fcn"};
this.def["rtIsInf"] = {file: "Plantv50_cpp.html",line:164,type:"fcn"};
this.def["rtIsInfF"] = {file: "Plantv50_cpp.html",line:170,type:"fcn"};
this.def["rtIsNaN"] = {file: "Plantv50_cpp.html",line:176,type:"fcn"};
this.def["rtIsNaNF"] = {file: "Plantv50_cpp.html",line:199,type:"fcn"};
this.def["rtGetInf"] = {file: "Plantv50_cpp.html",line:213,type:"fcn"};
this.def["rtGetInfF"] = {file: "Plantv50_cpp.html",line:237,type:"fcn"};
this.def["rtGetMinusInf"] = {file: "Plantv50_cpp.html",line:248,type:"fcn"};
this.def["rtGetMinusInfF"] = {file: "Plantv50_cpp.html",line:272,type:"fcn"};
this.def["rt_atan2d_snf"] = {file: "Plantv50_cpp.html",line:280,type:"fcn"};
this.def["step"] = {file: "Plantv50_cpp.html",line:317,type:"fcn"};
this.def["initialize"] = {file: "Plantv50_cpp.html",line:721,type:"fcn"};
this.def["DW"] = {file: "Plantv50_h.html",line:45,type:"type"};
this.def["ConstP"] = {file: "Plantv50_h.html",line:63,type:"type"};
this.def["ExtU"] = {file: "Plantv50_h.html",line:68,type:"type"};
this.def["ExtY"] = {file: "Plantv50_h.html",line:76,type:"type"};
this.def["rtU"] = {file: "Plantv50_h.html",line:86,type:"var"};
this.def["rtConstP"] = {file: "Plantv50_data_cpp.html",line:24,type:"var"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:57,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:58,type:"type"};
this.def["int64_T"] = {file: "rtwtypes_h.html",line:59,type:"type"};
this.def["uint64_T"] = {file: "rtwtypes_h.html",line:60,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:61,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:62,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:71,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:72,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:73,type:"type"};
this.def["ulonglong_T"] = {file: "rtwtypes_h.html",line:74,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:75,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:76,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:77,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:98,type:"type"};
}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_cpp.html"] = "../ert_main.cpp";
	this.html2Root["ert_main_cpp.html"] = "ert_main_cpp.html";
	this.html2SrcPath["Plantv50_cpp.html"] = "../Plantv50.cpp";
	this.html2Root["Plantv50_cpp.html"] = "Plantv50_cpp.html";
	this.html2SrcPath["Plantv50_h.html"] = "../Plantv50.h";
	this.html2Root["Plantv50_h.html"] = "Plantv50_h.html";
	this.html2SrcPath["Plantv50_data_cpp.html"] = "../Plantv50_data.cpp";
	this.html2Root["Plantv50_data_cpp.html"] = "Plantv50_data_cpp.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_cpp.html","Plantv50_cpp.html","Plantv50_h.html","Plantv50_data_cpp.html","rtwtypes_h.html"];
