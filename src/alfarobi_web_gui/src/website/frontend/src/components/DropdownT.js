import React from "react";
import Popper from "popper.js";
import CheckBox from "./CheckBox";
import { useState } from "react";
import ROSLIB from "roslib";

function DropdownT({ color, ros }) {
  const props = [
    "r_sho_r",
    "r_sho_p",
    "r_el",
    "r_hip_r",
    "r_hip_p",
    "r_hip_y",
    "r_knee",
    "r_ank_r",
    "r_ank_p",
    "head_tilt",
    "head_pan",
    "l_sho_r",
    "l_sho_p",
    "l_el",
    "l_hip_r",
    "l_hip_p",
    "l_hip_y",
    "l_knee",
    "l_ank_r",
    "l_ank_p",
  ];

  var setTorque = new ROSLIB.Topic({
    ros: ros,
    name: "/torque",
    messageType: "alfarobi_web_gui/Torque",
  });

  var torque = new ROSLIB.Message({
    joint_name: "",
    joint_state: true,
  });

  const [torqueState, setTorqueState] = useState({
    head_pan: true,
    head_tilt: true,
    l_ank_p: true,
    l_ank_r: true,
    l_hip_p: true,
    l_hip_r: true,
    l_hip_y: true,
    l_knee: true,
    l_sho_p: true,
    l_sho_r: true,
    l_el: true,
    r_ank_p: true,
    r_ank_r: true,
    r_hip_p: true,
    r_hip_r: true,
    r_hip_y: true,
    r_knee: true,
    r_sho_p: true,
    r_sho_r: true,
    r_el: true,
    target_time: true,
    pause_time: true,
  });
  const [dropdownPopoverShow, setDropdownPopoverShow] = React.useState(false);
  const btnDropdownRef = React.createRef();
  const popoverDropdownRef = React.createRef();
  const openDropdownPopover = () => {
    new Popper(btnDropdownRef.current, popoverDropdownRef.current, {
      placement: "bottom-start",
    });
    setDropdownPopoverShow(true);
  };
  const closeDropdownPopover = () => {
    setDropdownPopoverShow(false);
  };

  return (
    <div className="flex flex-wrap pl-4">
      <div className="inline-flex align-middle w-full">
        <button
          className="text-black w-[2vw] pl-2 text-left text-sm rounded shadow hover:bg-white outline-none focus:outline-none bg-secondary_bg"
          style={{ transition: "all .15s ease" }}
          type="button"
          ref={btnDropdownRef}
          onClick={() => {
            dropdownPopoverShow
              ? closeDropdownPopover()
              : openDropdownPopover();
          }}
        >
          ⌄
        </button>
        <div
          ref={popoverDropdownRef}
          className={
            (dropdownPopoverShow ? "block" : "hidden ") +
            "bg-white text-base w-[5vw] float-left py-2 list-none text-left rounded shadow-lg mt-1"
          }
          style={{ minWidth: "12rem" }}
        >
          {props.map((data, index) => (
            <div>
              <CheckBox
                ros={ros}
                joint={data}
                state={torqueState}
                setTorqueState={setTorqueState}
                message={setTorque}
              />
            </div>
          ))}
          <div className="bg-white">
            <button
              className="mt-2 mb-2 mx-4 p-2 px-10 bg-[#59E867] shadow-0 hover:bg-black text-black hover:text-[#B0ECFF] rounded-xl hover:cursor-pointer"
              onClick={() => {
                setTorqueState({
                  head_pan: true,
                  head_tilt: true,
                  l_ank_p: true,
                  l_ank_r: true,
                  l_hip_p: true,
                  l_hip_r: true,
                  l_hip_y: true,
                  l_knee: true,
                  l_sho_p: true,
                  l_sho_r: true,
                  l_el: true,
                  r_ank_p: true,
                  r_ank_r: true,
                  r_hip_p: true,
                  r_hip_r: true,
                  r_hip_y: true,
                  r_knee: true,
                  r_sho_p: true,
                  r_sho_r: true,
                  r_el: true,
                  target_time: true,
                  pause_time: true,
                });
                props.forEach((joint) => {
                  torque.joint_name = joint;
                  setTorque.publish(torque);
                });
              }}
              type="submit"
            >
              Enable All
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

export default DropdownT;
