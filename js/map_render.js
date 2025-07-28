/**
 * This file contains the client JavaScript logic, in charge of receiving the coordinates of the objects
 * (i.e. the vehicles) and their heading from the server (which in turn receives them from the S-LDM via UDP).
 * This script uses the received information in order to render a Leaflet JS map showing all the moving nodes.
 * The server->client communication is realized thanks to socket.io.
 */

// Constant for unavailable coordinates values
const VIS_LONLAT_INVALID = -80000;

// Constant for an unavailable heading value (VIS_HEADING_INVALID)
const VIS_HEADING_INVALID = 361;

// Car and circle icon indeces
const CAR_ICO_IDX = 0;
const CIRCLE_ICO_IDX = 1;
const GREEN_CIRCLE_ICO_IDX = 2;
const DETECTED_CAR_ICO_IDX = 3;
const DETECTED_PEDESTRIAN_ICO_IDX = 4;
const DETECTED_TRUCK_ICO_IDX = 5;
const ICY_ROAD_ICO_IDX = 6;
const GENERIC_DANGEROUS_SIGN_IDX = 7;

// Start socket.io()
const socket = io();

var map_rx = false;

// Markers array
var markers = [];
var eventMarkers = new Map();
// Markers icon array ('0' for carIcon, i.e. CAR_ICO_IDX, '1' for circleIcon, i.e. CIRCLE_ICO_IDX)
var markersicons = [];
var eventMarkersIcons = new Map();

// Map reference variable
var leafletmap = null;

// Create a new icon for the "car" markers
var icon_length = 102;
var icon_height = 277;
var icon_length_circle = 144;
var icon_height_circle = 143;
var icon_length_circle_green = 94;
var icon_height_circle_green = 94;
var icon_length_triangle_blue = 600;
var icon_height_triangle_blue = 600;

var icon_length_car = 225;
var icon_height_car = 300;

var icon_scale_factor = 10;
var carIcon = L.icon({
    iconUrl: './img/black_car2.png',

    iconSize:     [icon_length_car/icon_scale_factor, icon_height_car/icon_scale_factor], // size of the icon
    iconAnchor:   [icon_length_car/(icon_scale_factor*2), icon_height_car/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

var detectedTruckIcon = L.icon({
    iconUrl: './img/blue_truck.png',

    iconSize:     [icon_length/icon_scale_factor, icon_height/7], // size of the icon
    iconAnchor:   [icon_length/(icon_scale_factor*2), icon_height/(7*2)], // point of the icon which will correspond to marker's location
    popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

var detectedPedestrianIcon = L.icon({
    iconUrl: './img/pedestrian.png',

    iconSize:     [icon_length/icon_scale_factor, icon_height/icon_scale_factor], // size of the icon
    iconAnchor:   [icon_length/(icon_scale_factor*2), icon_height/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
    popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

var detectedCarIcon = L.icon({
    iconUrl: './img/detected_car6.png',

    iconSize:     [icon_length_car/icon_scale_factor, icon_height_car/icon_scale_factor], // size of the icon
    iconAnchor:   [icon_length_car/(icon_scale_factor*2), icon_height_car/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
    popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

var circleIcon = L.icon({
	iconUrl: './img/circle.png',

	iconSize:     [icon_length_circle/icon_scale_factor, icon_height_circle/icon_scale_factor], // size of the icon
	iconAnchor:   [icon_length_circle/(icon_scale_factor*2), icon_height_circle/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
    popupAnchor:  [0, 0] // point from which the popup shoPEDESTRIANuld open relative to the iconAnchor
});

var greenCircleIcon = L.icon({
	iconUrl: './img/green_circle.png',

	iconSize:     [icon_length_circle_green/icon_scale_factor, icon_height_circle_green/icon_scale_factor], // size of the icon
	iconAnchor:   [icon_length_circle_green/(icon_scale_factor*2), icon_height_circle_green/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

//Create a new icon for the "event" markers
var icon_length_icyroad = 512;
var icon_height_icyroad = 512;
var icon_height_dangerousIcon = 1275;
var icon_length_dangerousIcon = 1506;

var detectedIcyRoadIcon = L.icon({
	iconUrl: './img/icy_road.png',
	iconSize: 	[icon_length_icyroad/icon_scale_factor, icon_height_icyroad/icon_scale_factor],
	iconAnchor: [icon_length_icyroad/(icon_scale_factor*2), icon_height_icyroad/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor: [0, 0] // point from which the popup should open relative to the iconAnchor
});

var genericDangerousIcon = L.icon({
	iconUrl: './img/dangerous_sign.png',
	iconSize: 	[icon_length_dangerousIcon/icon_scale_factor, icon_height_dangerousIcon/icon_scale_factor],
	iconAnchor: [icon_length_dangerousIcon/(icon_scale_factor*2), icon_height_dangerousIcon/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor: [0, 0] // point from which the popup should open relative to the iconAnchor
})


// Receive the first message from the server
socket.on('message', (msg) => {
	if (msg == null) {
		document.getElementById('statusid').innerHTML = '<p>Waiting for a connection from the S-LDM</p>';
	} else {
		let msg_fields = msg.split(",");

		switch (msg_fields[0]) {
			// "map areas draw" message: "map_areas,<lat>,<lon>,<minlat>,<minlon>,<maxlat>,<maxlon>,<lat_ext_factor>,<lon_ext_factor>,<mapbox token>"
			case 'map_areas':
				if (map_rx === false) {
					if (msg_fields.length !== 10) {
						console.error("VehicleVisualizer: Error: received a corrupted map draw message from the server (map_areas type)");
					} else {
						console.info("VehicleVisualizer: The map will be drawn centered at: ", msg_fields[1], msg_fields[2]);
						console.info("VehicleVisualizer: displayed area: [",msg_fields[3],",",msg_fields[4],"],[",msg_fields[5],",",msg_fields[6],"]");
						document.getElementById('statusid').innerHTML = '';
						let mapbox_token;

						console.log(msg_fields[3]);

						if(msg_fields[9] != "none") {
							mapbox_token = msg_fields[9];
						} else {
							mapbox_token = null;
						}
						leafletmap = draw_map(parseFloat(msg_fields[1]), parseFloat(msg_fields[2]), parseFloat(msg_fields[3]), 
							parseFloat(msg_fields[4]), parseFloat(msg_fields[5]), parseFloat(msg_fields[6]), 
							parseFloat(msg_fields[7]), parseFloat(msg_fields[8]), mapbox_token);
						map_rx = true;
					}
				}
				break;

			// "map draw" message: "map,<lat>,<lon>,<mapbox token>"
			case 'map':
				if (map_rx === false) {
					if (msg_fields.length !== 4) {
						console.error("VehicleVisualizer: Error: received a corrupted map draw message from the server");
					} else {
						console.info("VehicleVisualizer: The map will be drawn centered at: ", msg_fields[1], msg_fields[2]);
						document.getElementById('statusid').innerHTML = '';
						let mapbox_token;

						console.log(msg_fields[3]);

						if(msg_fields[3] != "none") {
							mapbox_token = msg_fields[3];
						} else {
							mapbox_token = null;
						}
						leafletmap = draw_map(parseFloat(msg_fields[1]), parseFloat(msg_fields[2]), VIS_LONLAT_INVALID, 
							VIS_LONLAT_INVALID, VIS_LONLAT_INVALID, VIS_LONLAT_INVALID, 
							VIS_LONLAT_INVALID, VIS_LONLAT_INVALID, mapbox_token);
						map_rx = true;
					}
				}
				break;
			// "object update"/vehicle update message: "object,<unique object ID>,<lat>,<lon>,<stationtype>,<heading>"
			case 'object':
				if(msg_fields.length !== 6) {
					console.error("VehicleVisualizer: Error: received a corrupted object update message from the server.");
				} else {
					update_marker(leafletmap,msg_fields[1],parseFloat(msg_fields[2]),parseFloat(msg_fields[3]),parseInt(msg_fields[4]),parseFloat(msg_fields[5]));
				}
				break;
				
			case 'event':
				if(msg_fields.length !==6){
					console.error("VehicleVisualizer: Error: received a corrupted eventObject update message from the server.");
				} else {
					update_EventMarker(leafletmap,msg_fields[1],parseFloat(msg_fields[2]),parseFloat(msg_fields[3]),parseFloat(msg_fields[4]),parseInt(msg_fields[5]));
				}
				break;

			case 'objEventClean':
				if(msg_fields.length !== 2){
					console.error("VehicleVisualizer: Error: received a corrupted event clean message from the server.");
				} else {
					//console.log("Inside the else event clean"); // For debugging purposes
					let idEvent =  msg_fields[1];

					/*
					for (let key of eventMarkers.keys()) {
						console.log(`Comparing: '${idEvent}' (HEX: ${toHex(idEvent)}) === '${key}' (HEX: ${toHex(key)})`);
					}
					*/

					//idEvent = idEvent.trim();  // Rimuove spazi prima e dopo
					//idEvent = idEvent.normalize();  // Normalizza caratteri Unicode (se presenti)
					idEvent = idEvent.replace(/\0/g, "");

					for (let key of eventMarkers.keys()) {
						let cleanedKey = key.trim().normalize();
						if (idEvent === cleanedKey) {
							console.log("✅ Match trovato!");
						} //else {
							//console.log(`❌ Nessun match: '${idEvent}' !== '${cleanedKey}'`);
						//}
					}

					/*
					for (let key of eventMarkers.keys()) {
						console.log(`Tipo chiave: ${typeof key}, Valore: '${key}'`);
					}
					*/
					//console.log(idEvent in eventMarkers.keys())
					//console.log("TIPO: ",typeof idEvent);
					//console.log("Type: ",typeof eventMarkers.keys().next().value);
					//console.log("ID EVENT: ", idEvent);
					//console.log("MAP_KEYS", eventMarkers.keys());
					if(eventMarkers.has(String(idEvent))){
						//console.log("SIZE before delete", eventMarkers.size);
						console.log(`Remove marker whit ID: ${idEvent}`);
						let marker = eventMarkers.get(idEvent);
						marker.remove(); // Remove the marker from the map
						eventMarkers.delete(String(idEvent));
						console.log("Event marker removed");

						//For debugging purposes, print the current size of the eventMarkers map
						//console.log("SIZE after delete", eventMarkers.size);
						//console.log("All ID in the map:", Array.from(eventMarkers.keys()));
						//console.log("Actual size:", eventMarkers.size);
					} else {
						console.warn(`❌ Tentativo di rimuovere un ID non esistente: ${idEvent}`);
					}
				}
				break;

			// "object clean"/vehicle removal message: "objclean,<unique object ID>"
			case 'objclean':
				if(msg_fields.length !== 2) {
					console.error("VehicleVisualizer: Error: received a corrupted object clean message from the server.");
				} else {
					let id = parseInt(msg_fields[1]);
					if(id in markers) {
						markers[id].remove();
						delete markers[id];
					}
				}
				break;
			// This 'case' is added just for additional safety. As the server is shut down every time a "terminate" message
			// is received from the S-LDM and no "terminate" message is forwarded via socket.io, this point should never be
			// reached
			case 'terminate':
				console.log("The server has been terminated.");
				break;

			default:
				console.warn("VehicleVisualizer: Warning: unknown message type received from the server");
		}
	}
});


function toHex(str) {
	return Array.from(str).map(c => c.charCodeAt(0).toString(16)).join(" ");
}

function update_EventMarker(mapref,idEvent,lat,lon,ele,CauseCode){
	//console.log("Event marker update");
	console.log("Event ID: "+idEvent);
	if(mapref==null){
		console.error("VehicleVisualizer: null map reference when attempting to update an object")
	} else{
		console.log("EVENTI_MARKERS", eventMarkers);
		console.log("Tutti gli ID presenti:", Array.from(eventMarkers.keys()));
		console.log("SIZE",eventMarkers.size)

		if (!eventMarkers.has(String(idEvent))){
			console.log("New marker");
			let newmarkerEvent;
			let initial_icon;
			let initial_icon_idx;

			if(CauseCode === 6) {
				initial_icon = detectedIcyRoadIcon;
				initial_icon_idx = ICY_ROAD_ICO_IDX;
			} else {
				if (CauseCode === 7){
					initial_icon = genericDangerousIcon;
					initial_icon_idx = GENERIC_DANGEROUS_SIGN_IDX;
				}
			}
			newmarkerEvent = L.marker([lat,lon], {icon: initial_icon}).addTo(mapref);
			newmarkerEvent.bindPopup("ID: " + idEvent + " - Cause Code: " + CauseCode);

			eventMarkers.set(String(idEvent),newmarkerEvent);
			eventMarkersIcons.set(String(idEvent),initial_icon_idx);
			console.log("END NEW marker\n", idEvent);
		} else {
			console.log("Marker already exists", idEvent);
			let markerEvent = eventMarkers.get(String(idEvent));
			markerEvent.setLatLng([lat,lon]);

			markerEvent.setPopupContent("EventID: "+idEvent+ " Cause Code:" +CauseCode);
			if (CauseCode === 6 && eventMarkersIcons.get(String(idEvent)) !== ICY_ROAD_ICO_IDX){
				markerEvent.setIcon(detectedIcyRoadIcon);
				eventMarkersIcons.set(idEvent, ICY_ROAD_ICO_IDX);
			} else if (CauseCode === 7 && eventMarkersIcons.get(String(idEvent)) !== GENERIC_DANGEROUS_SIGN_IDX){
				markerEvent.setIcon(genericDangerousIcon);
				eventMarkersIcons.set(idEvent, GENERIC_DANGEROUS_SIGN_IDX);
			}
			console.log("END marker already exists\n", idEvent);
		}
	}
}

// This function is used to update the position (and heading/rotation) of a marker/moving object on the map
function update_marker(mapref,id,lat,lon,stationtype,heading)
{
	if(mapref == null) {
		console.error("VehicleVisualizer: null map reference when attempting to update an object")
	} else {
		// If the object ID has never been seen before, create a new marker
		if(!(id in markers)) {
			let newmarker;
			let initial_icon;
			let initial_icon_idx;

			if(stationtype === 0) {
                initial_icon = detectedCarIcon;
                initial_icon_idx = DETECTED_CAR_ICO_IDX;
			} else {
				// Set a circular icon when the heading is not available, otherwise use the regular carIcon (i.e. for the time being, a triangle)
				if(heading >= VIS_HEADING_INVALID) {
					initial_icon = circleIcon;
					initial_icon_idx = CIRCLE_ICO_IDX;
				} else {
					switch (stationtype) {
						case 1:
						case 110:
							initial_icon = detectedPedestrianIcon;
							initial_icon_idx = DETECTED_PEDESTRIAN_ICO_IDX;
							break;
						case 115:
							initial_icon = detectedCarIcon;
							initial_icon_idx = DETECTED_CAR_ICO_IDX;
							break;
						case 117:
							initial_icon = detectedTruckIcon;
							initial_icon_idx = DETECTED_TRUCK_ICO_IDX;
							break;
						default:
							initial_icon = carIcon;
							initial_icon_idx = CAR_ICO_IDX;
					}
				}
			}

			// Attempt to use an icon marker
			newmarker = L.marker([lat,lon], {icon: initial_icon}).addTo(mapref);

            if(stationtype === 110 || stationtype === 1){
               newmarker.setRotationAngle(0);
            }else {
               newmarker.setRotationAngle(heading);
            }


            //newmarker.setRotationAngle(heading);

			// Old circle marker (no more used)
			// newmarker = L.circleMarker([lat,lon],{radius: 8, fillColor: "#c48612", color: "#000000", weight: 1, opacity: 1, fillOpacity: 0.8}).addTo(mapref);
			// Set the initial popup value (if the heading is invalid/unavailable, do not specify any value in degrees
			if(heading < VIS_HEADING_INVALID) {
				newmarker.bindPopup("ID: "+id+" - Heading: "+heading+" deg");
			} else {
				newmarker.bindPopup("ID: "+id+" - Heading: unavailable");
			}
			markers[id]=newmarker;
			markersicons[id]=initial_icon_idx;
		// If the object ID has already been seen before, just update its attributes (i.e. position, rotation angle and popup content)
		} else {
			let marker = markers[id];
			marker.setLatLng([lat,lon]);
            if(stationtype !== 110 && stationtype !== 1){
               marker.setRotationAngle(heading);
            }

            //marker.setRotationAngle(heading);
			// Update the popup content if the heading value becomes invalid/unavailable after being available
			// or if the heading value if actually available (to specify the most up-to-date heading value)
			if(heading >= VIS_HEADING_INVALID && markersicons[id] === CAR_ICO_IDX) {
				marker.setPopupContent("ID: "+id+" - Heading: unavailable");
			} else if(heading < VIS_HEADING_INVALID) {
				marker.setPopupContent("ID: "+id+" - Heading: "+heading+" deg");
			}

			if(stationtype !== 0) {
				// If the heading becomes unavailable or invalid (but it was not before), change the icon of the vehicle to a circle
				if(heading >= VIS_HEADING_INVALID && (markersicons[id] === CAR_ICO_IDX || markersicons[id] === GREEN_CIRCLE_ICO_IDX)) {
					marker.setIcon(circleIcon);
					markersicons[id] = CIRCLE_ICO_IDX;
                } else {
                    // If the heading becomes available after being unavailable, change the icon of the vehicle to the "car" icon
                    if(heading < VIS_HEADING_INVALID && (markersicons[id] === CIRCLE_ICO_IDX || markersicons[id] === GREEN_CIRCLE_ICO_IDX)) {
						switch (stationtype) {
							case 1:
							case 110:
								marker.setIcon(detectedPedestrianIcon);
								markersicons[id] = DETECTED_PEDESTRIAN_ICO_IDX;
								break;
							case 115:
								marker.setIcon(detectedCarIcon);
								markersicons[id] = DETECTED_CAR_ICO_IDX;
								break;
							case 117:
								marker.setIcon(detectedTruckIcon);
								markersicons[id] = DETECTED_TRUCK_ICO_IDX;
								break;
							default:
								marker.setIcon(carIcon);
								markersicons[id] = CAR_ICO_IDX;
						}
                    }
                }
			} else {
				if(markersicons[id] === CAR_ICO_IDX || markersicons[id] === CIRCLE_ICO_IDX) {
					marker.setIcon(greenCircleIcon);
					markersicons[id] = GREEN_CIRCLE_ICO_IDX;
				}
			}
		}
	}
}

// This function is used to draw the whole map at the beginning, on which vehicles will be placed
// It expects as arguments the lat and lon value where the map should be centered
function draw_map(lat,lon,minlat,minlon,maxlat,maxlon,lat_ext_factor,lon_ext_factor,mapbox_token) {
	let standardlayer;

	// If no Mapbox token is specified, create a basic view layer based on OpenStreetMap (occasional use only! Heavy usage is forbidden!)
	if(mapbox_token == null) {
		standardlayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors | <a href="https://www.openstreetmap.org/fixthemap">Report a problem in the map</a>',
			id: 'mapbox/streets-v11',
			// tileSize: 512,
			// zoomOffset: -1,
		});
	}

	// Create standard (street view), hybrid and satellite layers based on Mapbox
	// They are normally not enabled as you need an access token to use them
	let hybridlayer, satellitelayer;

	if(mapbox_token != null) {
		hybridlayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=' + mapbox_token, {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.mapbox.com/about/maps/">Mapbox</a> | S-LDM vehicle visualizer hybrid view',
			id: 'mapbox/satellite-streets-v11',
			tileSize: 512,
			zoomOffset: -1,
		});

		hybridlayer.setOpacity(0.7);

		satellitelayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=' + mapbox_token, {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.mapbox.com/about/maps/">Mapbox</a> | S-LDM vehicle visualizer satellite view',
			id: 'mapbox/satellite-v9',
			tileSize: 512,
			zoomOffset: -1,
		});

		satellitelayer.setOpacity(0.7);

		standardlayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=' + mapbox_token, {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.mapbox.com/about/maps/">Mapbox</a> | S-LDM vehicle visualizer streets view',
			id: 'mapbox/streets-v11',
			tileSize: 512,
			zoomOffset: -1,
		});
	}

	// Main map object creation (default layer: standardlayer, i.e. OpenStreetMap or Mapbox street view)
	var mymap = L.map('mapid', {
		center: [lat, lon],
		zoom: 17,
		layers: standardlayer
	});

	// Add all the layers to the map, adding a control button to dynamically change the current map layer, if more than one layer can be used
	if (mapbox_token != null) {
		let basemaps = {
			"Street view": standardlayer,
			"Hybrid view": hybridlayer,
			"Satellite view": satellitelayer,
		};

		L.control.layers(basemaps).addTo(mymap);

		console.log("A Mapbox token has been specified. Multiple layers will be available.")
	}

	if(minlat!==VIS_LONLAT_INVALID && minlon!==VIS_LONLAT_INVALID && maxlat!==VIS_LONLAT_INVALID && maxlon!==VIS_LONLAT_INVALID) {
		L.rectangle([[minlat,minlon],[maxlat,maxlon]], {color: 'red', fill: false}).addTo(mymap);

		if(lat_ext_factor!==VIS_LONLAT_INVALID && lon_ext_factor!==VIS_LONLAT_INVALID) {
			L.rectangle([[minlat-lat_ext_factor,minlon-lon_ext_factor],[maxlat+lat_ext_factor,maxlon+lon_ext_factor]], {color: 'green', fill: false}).addTo(mymap);
		}
	}

	// Print on the console that the map has been successfully rendered
	console.log("VehicleVisualizer: Map Created!")

	return mymap;
}
