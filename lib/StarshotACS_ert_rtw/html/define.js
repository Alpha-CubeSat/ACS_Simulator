function CodeDefine() { 
this.def = new Array();
this.def["ert_main.cpp:rtObj"] = {file: "ert_main_cpp.html",line:26,type:"var"};
this.def["rt_OneStep"] = {file: "ert_main_cpp.html",line:40,type:"fcn"};
this.def["main"] = {file: "ert_main_cpp.html",line:77,type:"fcn"};
this.def["step"] = {file: "StarshotACS_cpp.html",line:24,type:"fcn"};
this.def["initialize"] = {file: "StarshotACS_cpp.html",line:406,type:"fcn"};
this.def["getRTM"] = {file: "StarshotACS_cpp.html",line:430,type:"fcn"};
this.def["RT_MODEL"] = {file: "StarshotACS_h.html",line:39,type:"type"};
this.def["DW"] = {file: "StarshotACS_h.html",line:46,type:"type"};
this.def["ConstB"] = {file: "StarshotACS_h.html",line:51,type:"type"};
this.def["ConstP"] = {file: "StarshotACS_h.html",line:59,type:"type"};
this.def["ExtU"] = {file: "StarshotACS_h.html",line:65,type:"type"};
this.def["ExtY"] = {file: "StarshotACS_h.html",line:71,type:"type"};
this.def["rtU"] = {file: "StarshotACS_h.html",line:88,type:"var"};
this.def["rtConstB"] = {file: "StarshotACS_data_cpp.html",line:24,type:"var"};
this.def["rtConstP"] = {file: "StarshotACS_data_cpp.html",line:40,type:"var"};
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
	this.html2SrcPath["StarshotACS_cpp.html"] = "../StarshotACS.cpp";
	this.html2Root["StarshotACS_cpp.html"] = "StarshotACS_cpp.html";
	this.html2SrcPath["StarshotACS_h.html"] = "../StarshotACS.h";
	this.html2Root["StarshotACS_h.html"] = "StarshotACS_h.html";
	this.html2SrcPath["StarshotACS_data_cpp.html"] = "../StarshotACS_data.cpp";
	this.html2Root["StarshotACS_data_cpp.html"] = "StarshotACS_data_cpp.html";
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
"ert_main_cpp.html","StarshotACS_cpp.html","StarshotACS_h.html","StarshotACS_data_cpp.html","rtwtypes_h.html"];
