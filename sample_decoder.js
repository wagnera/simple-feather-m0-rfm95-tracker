function Decoder(bytes, port) {
    var decoded = {};
  if (port === 1) {
    var i = 0;
    decoded.latitude = (bytes[i++]) + (bytes[i++]<<8) + (bytes[i++]<<16) + (bytes[i++]<<24);
    decoded.latitude = (decoded.latitude / 1e7) - 90;
    decoded.longitude = (bytes[i++]) + (bytes[i++]<<8) + (bytes[i++]<<16) + (bytes[i++]<<24);
    decoded.longitude = (decoded.longitude / 1e7) - 180;
    decoded.altitude = bytes[i++] + (bytes[i++]<<8);
    decoded.altitude = decoded.altitude - 1000;
    decoded.hdop = bytes[i++];
    decoded.hdop = decoded.hdop /10.0;
  }
  
  var decodedPayload = {
    "latitude": decoded.latitude,
    "altitude": decoded.altitude,
    "longitude": decoded.longitude,
    "hdop": decoded.hdop
  };
  // END TODO

  return decodedPayload
}