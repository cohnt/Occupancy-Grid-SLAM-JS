/////////////////
/// CONSTANTS ///
/////////////////

var canvasSize = [0.49, 0.7]; //Size of each canvas, as a fraction of the total page width and height, respectively
var worldScale = 20; //The width of the entire browser viewport (in meters), determining the scale of the displays

////////////////////////
/// GLOBAL VARIABLES ///
////////////////////////

var worldCanvas;
var mapCanvas;

///////////////
/// CLASSES ///
///////////////

//

/////////////////
/// FUNCTIONS ///
/////////////////

function setup() {
	worldCanvas = document.getElementById("worldCanvas");
	mapCanvas = document.getElementById("mapCanvas");

	worldCanvas.setAttribute("width", String(window.innerWidth * canvasSize[0]) + "px");
	worldCanvas.setAttribute("height", String(window.innerHeight * canvasSize[1]) + "px");
	mapCanvas.setAttribute("width", String(window.innerWidth * canvasSize[0]) + "px");
	mapCanvas.setAttribute("height", String(window.innerHeight * canvasSize[1]) + "px");
}

/////////////////////
/// EXECUTED CODE ///
/////////////////////

requestAnimationFrame(setup);