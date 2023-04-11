import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";
import axios from "axios";
import AppBar from "../components/AppBar";
import DropdownBM from "../components/DropdownBM";
import DropdownT from "../components/DropdownT";
import InputCard from "../components/InputCard";
import robotBody from "../assets/robot_body_joint.png";
import robotStore from "../redux/GlobalState";
import { useSelector } from "react-redux";
import { useParams } from "react-router-dom";
import { connect } from "react-redux";

function InitialPoses(props) {
  const { robot } = useParams();
  const robots = useSelector(() => robotStore);
  const [loaded, setLoaded] = useState(false);
  const [edited, setEdited] = useState({})
  const getInit = async () => {
    try {
      const response = await axios.get(
        `http://10.42.0.91:5000/init?robot=${robot}`
      );
      // console.log(response.data);
      props.handleInit("GET_INIT", robot, response.data.data);
    } catch (error) {
      console.log(error);
    }
    setLoaded(true);
  };

  useEffect(() => {
    getInit();
  }, []);

  const saveInit = async (e) => {
    e.preventDefault();
    try {
      instructionMsg.data = "apply";
      instruction.publish(instructionMsg); 
      const robotState = robots.getState()[`${robot}`];
      twist = robotState;
      console.log(twist);
      cmdVel.publish(twist);
      await axios.post(`http://10.42.0.91:5000/init?robot=${robot}`, robotState);
      getInit();
      setLoaded(false);
    } catch (error) {
      console.log(error);
    }
  };

  var ros = new ROSLIB.Ros({
    url: "ws://10.42.0.91:6969",
  });

  ros.on("connection", function () {
    console.log("Connected to websocket server.");
  });

  ros.on("error", function (error) {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", function () {
    console.log("Connection to websocket server closed.");
  });

  var listener = new ROSLIB.Topic({
    ros: ros,
    name: "initial_pose/joint_value",
    messageType: "alfarobi_web_gui/Sequencer",
  });

  listener.subscribe(function (message) {
    setEdited(message);
  });

  var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: "initial_pose/joint_value_web",
    messageType: "alfarobi_web_gui/Sequencer",
  });

  var instruction = new ROSLIB.Topic({
    ros: ros,
    name: "initial_pose/web_button",
    messageType: "std_msgs/String",
  });

  var instructionMsg = new ROSLIB.Message({
    data: "",
  });

  var twist = new ROSLIB.Message();

  return (
    <div>
      {loaded && (
        <div className="flex flex-col h-screen bg-secondary_bg">
          <AppBar />
          <div className="flex flex-row">
            <div className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-4 bg-primary_bg rounded-xl">
              <p className="text-white text-1xl">Body module : </p>
              <DropdownBM color={"white"} sequence={["JATUH_DEPAN", "JATUH_BELAKANG", "JATUH_KANAN", "JATUH_KIRI"]} />
            </div>
            <div className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-4 bg-primary_bg rounded-xl">
              <p className="text-white text-1xl">Torque(s)</p>
              <DropdownT color={"white"} ros={ros} topicNameTorque={"/torque"} topicNameInstruction={"/initial_pose/web_button"}/>
            </div>
            <div
              className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-10 bg-[#04C3FF] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
              onClick={saveInit}
            >
              <p>Apply!</p>
            </div>
            <div
              className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-10 bg-[#04C3FF] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
              onClick={() => {
                instructionMsg.data = "save";
                instruction.publish(instructionMsg);
              }}
            >
              {/* init_pose save apply refresh */}
              <p>Save!</p>
            </div>
            <div
              className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-10 bg-[#04C3FF] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
              onClick={() => {
                instructionMsg.data = "init_pose";
                instruction.publish(instructionMsg);
              }}
            >
              {/* init_pose save apply refresh */}
              <p>init_pose!</p>
            </div>

            <div
              className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-10 bg-[#04C3FF] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
              onClick={() => {
                instructionMsg.data = "refresh";
                instruction.publish(instructionMsg);
              }}
            >
              {/* init_pose save apply refresh */}
              <p>Refresh!</p>
            </div>
          </div>
          <div className="flex flex-row w-screen justify-center">
            <div className="flex flex-col w-[31vw] mt-[5vw]">
              <InputCard robot={robot} titles={["r_sho_r", "r_sho_p"]} init_joint={edited} />
              <InputCard robot={robot} titles={["r_el"]} init_joint={edited} />
              <InputCard
                robot={robot}
                titles={["r_hip_r", "r_hip_p", "r_hip_y"]}
                init_joint={edited}
              />
              <InputCard robot={robot} titles={["r_knee"]} init_joint={edited}/>
              <InputCard robot={robot} titles={["r_ank_r", "r_ank_p"]} init_joint={edited}/>
            </div>
            <div className="grid justify-center items-center">
              <img src={robotBody} alt="robot" className="w-[32vw] h-[74vh]" />
            </div>
            <div className="flex flex-col w-[31vw] text-center space-x-2">
              <div className="flex flex-col w-[31vw]">
                <InputCard robot={robot} titles={["head_tilt", "head_pan"]} init_joint={edited}/>
                <InputCard robot={robot} titles={["l_sho_r", "l_sho_p"]} init_joint={edited}/>
                <InputCard robot={robot} titles={["l_el"]} init_joint={edited}/>
                <InputCard
                  robot={robot}
                  titles={["l_hip_r", "l_hip_p", "l_hip_y"]}
                  init_joint={edited}
                />
                <InputCard robot={robot} titles={["l_knee"]} init_joint={edited}/>
                <InputCard robot={robot} titles={["l_ank_r", "l_ank_p"]} init_joint={edited}/>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

const mapStateToProps = (state) => {
  return {
    state: state,
  };
};

const mapDispatchToProps = (dispatch) => {
  return {
    handleInit: (type, robot, value) => {
      console.log("handleInit");
      dispatch({ type: type, robot: robot, value: value });
    },
  };
};

export default connect(mapStateToProps, mapDispatchToProps)(InitialPoses);
