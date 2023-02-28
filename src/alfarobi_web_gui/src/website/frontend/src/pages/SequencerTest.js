import React from "react";
import ROSLIB from "roslib";
import { useState, useEffect } from "react";

function SequencerTest() {
  const [sequence, setSequence] = useState({
    HEAD_PAN: 0.0,
    HEAD_TILT: 0.0,
    L_ANK_P: 0.0,
    L_ANK_R: 0.0,
    L_HIP_P: 0.0,
    L_HIP_R: 0.0,
    L_HIP_Y: 0.0,
    L_KNEE: 0.0,
    L_SHO_P: 0.0,
    L_SHO_R: 0.0,
    L_EL: 0.0,
    R_ANK_P: 0.0,
    R_ANK_R: 0.0,
    R_HIP_P: 0.0,
    R_HIP_R: 0.0,
    R_HIP_Y: 0.0,
    R_KNEE: 0.0,
    R_SHO_P: 0.0,
    R_SHO_R: 0.0,
    R_EL: 0.0,
    TARGET_TIME: 0.0,
    STOP_TIME: 0.0,
  });

  const joints = [
    "HEAD_PAN",
    "HEAD_TILT",
    "L_ANK_P",
    "L_ANK_R",
    "L_HIP_P",
    "L_HIP_R",
    "L_HIP_Y",
    "L_KNEE",
    "L_SHO_P",
    "L_SHO_R",
    "L_EL",
    "R_ANK_P",
    "R_ANK_R",
    "R_HIP_P",
    "R_HIP_R",
    "R_HIP_Y",
    "R_KNEE",
    "R_SHO_P",
    "R_SHO_R",
    "R_EL",
    "TARGET_TIME",
    "STOP_TIME",
  ];

  console.log("value is:", sequence);

  var ros = new ROSLIB.Ros({
    url: "ws://localhost:6969",
  });

  useEffect(() => {});

  ros.on("connection", function () {
    console.log("Connected to websocket server.");
  });

  ros.on("error", function (error) {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", function () {
    console.log("Connection to websocket server closed.");
  });

  var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: "/Sequencer",
    messageType: "alfarobi_web_gui/Sequencer",
  });

  var sequencer = new ROSLIB.Message({
    HEAD_PAN: parseFloat(sequence["HEAD_PAN"]),
    HEAD_TILT: parseFloat(sequence["HEAD_TILT"]),
    L_ANK_P: parseFloat(sequence["L_ANK_P"]),
    L_ANK_R: parseFloat(sequence["L_ANK_R"]),
    L_HIP_P: parseFloat(sequence["L_HIP_P"]),
    L_HIP_R: parseFloat(sequence["L_HIP_R"]),
    L_HIP_Y: parseFloat(sequence["L_HIP_Y"]),
    L_KNEE: parseFloat(sequence["L_KNEE"]),
    L_SHO_P: parseFloat(sequence["L_SHO_P"]),
    L_SHO_R: parseFloat(sequence["L_SHO_R"]),
    L_EL: parseFloat(sequence["L_EL"]),
    R_ANK_P: parseFloat(sequence["R_ANK_P"]),
    R_ANK_R: parseFloat(sequence["R_ANK_R"]),
    R_HIP_P: parseFloat(sequence["R_HIP_P"]),
    R_HIP_R: parseFloat(sequence["R_HIP_R"]),
    R_HIP_Y: parseFloat(sequence["R_HIP_Y"]),
    R_KNEE: parseFloat(sequence["R_KNEE"]),
    R_SHO_P: parseFloat(sequence["R_SHO_P"]),
    R_SHO_R: parseFloat(sequence["R_SHO_R"]),
    R_EL: parseFloat(sequence["R_EL"]),
    TARGET_TIME: parseFloat(sequence["TARGET_TIME"]),
    STOP_TIME: parseFloat(sequence["STOP_TIME"]),
  });
  //sequencer = parseFloat(edited);

  var listener = new ROSLIB.Topic({
    ros: ros,
    name: "/Sequencer",
    messageType: "alfarobi_web_gui/Sequencer",
  });

  listener.subscribe(function (message) {
    console.log("Received message " + listener.name + ": " + message);
  });

  return (
    <div>
      <form>
        {joints.map((data) => (
          <div>
            <p>{data}</p>
            <input
              type="number"
              step="0.01"
              className="w-[6vw] h-[2vh] ml-[1.8vw]"
              value={sequence[`${data}`]}
              onChange={(event) => {
                setSequence({
                  ...sequence,
                  [`${data}`]: parseFloat(event.target.value),
                });
              }}
            />
          </div>
        ))}
      </form>
      <button
        onClick={() => {
          console.log(sequencer);
          cmdVel.publish(sequencer);
        }}
        type="submit"
      >
        Send
      </button>
    </div>
  );
}

export default SequencerTest;
