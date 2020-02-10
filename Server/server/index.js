const ttn = require("ttn");
const appID = "48a0f671b3a833e85e4b5b96691b1d62";
const accessKey = "ttn-account-v2.Q7AzH_1yLbzwGrdGGu2YM0BYaR1iPv4vspr1yfuSzVo";
const devID = "48a0f671b3a833e85e4b5b96691b1d62";

const admin = require("firebase-admin");
const serviceAccount = require("./serviceAccountKey.json");

admin.initializeApp({
  credential: admin.credential.cert(serviceAccount),
  databaseURL: "https://fir-c5cc6.firebaseio.com"
});


var ref = admin.database().ref("DataTTN");
var sendDataFirebase = ref.child('messages');

ttn.data(appID, accessKey)
  .then(function (client) {
    client.on("uplink", function (devID, payload) {
      console.log("Received uplink from ", devID)
      let dataGPS = payload.payload_fields;
      console.log(dataGPS);
      sendDataFirebase.update(dataGPS);
    })
  })
  .catch(function (error) {
    console.error("Error", error)
    process.exit(1)
  });