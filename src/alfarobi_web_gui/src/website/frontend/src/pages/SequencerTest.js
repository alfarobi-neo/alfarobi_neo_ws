import React from "react";
import AppBar from "../components/AppBar";
import ROSLIB from "roslib";
import { useState, useEffect } from "react";

function SequencerTest() {
  const [sequence, setSequence] = useState({
    head_pan: 0.0,
    head_tilt: 0.0,
    l_ank_p: 0.0,
    l_ank_l: 0.0,
    l_hip_p: 0.0,
    l_hip_r: 0.0,
    l_hip_y: 0.0,
    l_knee: 0.0,
    l_sho_p: 0.0,
    l_sho_r: 0.0,
    l_el: 0.0,
    r_ank_p: 0.0,
    r_ank_r: 0.0,
    r_hip_p: 0.0,
    r_hip_r: 0.0,
    r_hip_y: 0.0,
    r_knee: 0.0,
    r_sho_p: 0.0,
    r_sho_r: 0.0,
    r_el: 0.0,
    target_time: 0.0,
    pause_time: 0.0,
  });

  const [sequenceArr, setSequenceArr] = useState({
    SEQUENCE_NAME: "",
    SEQUENCE: [],
  });

  const joints = [
    "head_pan",
    "head_tilt",
    "l_ank_p",
    "l_ank_r",
    "l_hip_p",
    "l_hip_r",
    "l_hip_y",
    "l_knee",
    "l_sho_p",
    "l_sho_r",
    "l_el",
    "r_ank_p",
    "r_ank_r",
    "r_hip_p",
    "r_hip_r",
    "r_hip_y",
    "r_knee",
    "r_sho_p",
    "r_sho_r",
    "r_el",
    "target_time",
    "stop_time",
  ];

  // console.log("value is:", sequence);

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

  var seq = new ROSLIB.Topic({
    ros: ros,
    name: "/Sequencer",
    messageType: "alfarobi_web_gui/Sequencer",
  });

  var seqArr = new ROSLIB.Topic({
    ros: ros,
    name: "/SequencerArr",
    messageType: "alfarobi_web_gui/SequencerArr",
  });

  var sequencer = new ROSLIB.Message({
    HEAD_PAN: parseFloat(sequence["head_pan"]),
    HEAD_TILT: parseFloat(sequence["head_tilt"]),
    L_ANK_P: parseFloat(sequence["l_ank_p"]),
    L_ANK_R: parseFloat(sequence["l_ank_r"]),
    L_HIP_P: parseFloat(sequence["l_hip_p"]),
    L_HIP_R: parseFloat(sequence["l_hip_r"]),
    L_HIP_Y: parseFloat(sequence["l_hip_y"]),
    L_KNEE: parseFloat(sequence["l_knee"]),
    L_SHO_P: parseFloat(sequence["l_sho_p"]),
    L_SHO_R: parseFloat(sequence["l_sho_r"]),
    L_EL: parseFloat(sequence["l_el"]),
    R_ANK_P: parseFloat(sequence["r_ank_p"]),
    R_ANK_R: parseFloat(sequence["r_ank_r"]),
    R_HIP_P: parseFloat(sequence["r_hip_p"]),
    R_HIP_R: parseFloat(sequence["r_hip_r"]),
    R_HIP_Y: parseFloat(sequence["r_hip_y"]),
    R_KNEE: parseFloat(sequence["r_knee"]),
    R_SHO_P: parseFloat(sequence["r_sho_p"]),
    R_SHO_R: parseFloat(sequence["r_sho_r"]),
    R_EL: parseFloat(sequence["r_el"]),
    TARGET_TIME: parseFloat(sequence["target_time"]),
    STOP_TIME: parseFloat(sequence["stop_time"]),
  });

  var sequencerArr = new ROSLIB.Message({
    SEQUENCE_NAME: "NGACENG",
    SEQUENCE: [],
  });

  var seqListener = new ROSLIB.Topic({
    ros: ros,
    name: "/Sequencer",
    messageType: "alfarobi_web_gui/Sequencer",
  });

  seqListener.subscribe(function (message) {
    // console.log("Received message " + seqListener.name + ": " + message);
  });

  var seqArrListener = new ROSLIB.Topic({
    ros: ros,
    name: "/SequencerArr",
    messageType: "alfarobi_web_gui/SequencerArr",
  });

  seqArrListener.subscribe(function (message) {
    console.log("DAPET BOS " + seqArrListener.name + ": " + message);
  });

  return (
    <div className="flex flex-col h-screen bg-secondary_bg">
      <AppBar />
      <div className="flex flex-row">
        <form className="bg-primary_bg mt-5 mb-1 p-2 rounded-lg">
          {joints.map((data) => (
            <div className="flex flex-row bg-secondary_bg mx-1 py-0.5  mt-1 rounded">
              <div className="flex flex-row items-center text-center">
                <p className="w-[7vw] text-[12px]">{data}</p>
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
            </div>
          ))}
        </form>
      </div>
      <div className="flex flex-row">
        <button
          className="mt-4 mb-12 mx-4 p-2 px-10 bg-[#04C3FF] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
          onClick={() => {
            var myArr = sequenceArr.SEQUENCE;
            myArr.push(sequence);
            setSequenceArr({ SEQUENCE_NAME: "Test", SEQUENCE: myArr });
          }}
        >
          Save Sequence
        </button>
        <button
          className="mt-4 mb-12 mx-4 p-2 px-10 bg-[#59E867] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
          onClick={() => {
            sequencerArr.SEQUENCE_NAME = sequenceArr.SEQUENCE_NAME;
            sequencerArr.SEQUENCE = sequenceArr.SEQUENCE;
            seq.publish(sequencer);
            seqArr.publish(sequencerArr);
          }}
          type="submit"
        >
          Send
        </button>
      </div>
    </div>
  );
}

export default SequencerTest;
