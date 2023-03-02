import React, { useEffect, useState } from "react";
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

  const getInit = async () => {
    try {
      const response = await axios.get(
        `http://127.0.0.1:5000/init?robot=${robot}`
      );
      console.log(response.data);
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
      const robotState = robots.getState()[`${robot}`];
      console.log(robotState);
      await axios.post(`http://127.0.0.1:5000/init?robot=${robot}`, robotState);
      getInit();
      setLoaded(false);
    } catch (error) {
      console.log(error);
    }
  };
  return (
    <div>
      {loaded && (
        <div className="flex flex-col h-screen bg-secondary_bg">
          <AppBar />
          <div className="flex flex-row">
            <div className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-4 bg-primary_bg rounded-xl">
              <p className="text-white text-1xl">Body module : </p>
              <DropdownBM color={"white"} />
            </div>
            <div className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-4 bg-primary_bg rounded-xl">
              <p className="text-white text-1xl">Torque(s)</p>
              <DropdownT color={"white"} />
              <button
                className="text-black w-[3vw] pl-2 text-left text-sm rounded shadow ml-2 hover:bg-white outline-none focus:outline-none bg-[#59E867]"
                style={{ transition: "all .15s ease" }}
                type="button"
                onClick={() => {}}
              >
                ON
              </button>
              <button
                className="text-black w-[3vw] pl-2 text-left text-sm rounded shadow ml-2 hover:bg-white outline-none focus:outline-none bg-[#DC5047]"
                style={{ transition: "all .15s ease" }}
                type="button"
                onClick={() => {}}
              >
                OFF
              </button>
            </div>
            <div
              className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-10 bg-[#04C3FF] hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
              onClick={saveInit}
            >
              <p>SAVE!</p>
            </div>
          </div>
          <div className="flex flex-row w-screen justify-center">
            <div className="flex flex-col w-[31vw] mt-[5vw]">
              <InputCard robot={robot} titles={["r_sho_roll", "r_sho_pitch"]} />
              <InputCard robot={robot} titles={["r_el_pitch"]} />
              <InputCard
                robot={robot}
                titles={["r_hip_roll", "r_hip_pitch", "r_hip_yaw"]}
              />
              <InputCard robot={robot} titles={["r_knee_pitch"]} />
              <InputCard robot={robot} titles={["r_ank_roll", "r_ank_pitch"]} />
            </div>
            <div className="grid justify-center items-center">
              <img src={robotBody} alt="robot" className="w-[32vw] h-[74vh]" />
            </div>
            <div className="flex flex-col w-[31vw] text-center space-x-2">
              <div className="flex flex-col w-[31vw]">
                <InputCard robot={robot} titles={["head_tilt", "head_pan"]} />
                <InputCard
                  robot={robot}
                  titles={["l_sho_roll", "l_sho_pitch"]}
                />
                <InputCard robot={robot} titles={["l_el_pitch"]} />
                <InputCard
                  robot={robot}
                  titles={["l_hip_roll", "l_hip_pitch", "l_hip_yaw"]}
                />
                <InputCard robot={robot} titles={["l_knee_pitch"]} />
                <InputCard
                  robot={robot}
                  titles={["l_ank_roll", "l_ank_pitch"]}
                />
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
