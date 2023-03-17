import React from "react";
import { useState } from "react";
import ROSLIB from "roslib";

function CheckBox({ joint, ros, state, setTorqueState, message }) {
  const handleChange = () => {
    setTorqueState({ ...state, [`${joint}`]: !state[`${joint}`] });
  };

  var torque = new ROSLIB.Message({
    joint_name: joint,
    joint_state: state[`${joint}`],
  });

  return (
    <label>
      <div className="flex flex-row text-sm pl-4 font-normal whitespace-no-wrap bg-white text-black">
        <input
          className="mx-2"
          type="checkbox"
          checked={state[`${joint}`]}
          onChange={() => {
            handleChange();
            torque.joint_state = !state[`${joint}`];
            console.log(torque);
            message.publish(torque);
          }}
        />
        <p>{joint}</p>
      </div>
    </label>
  );
}

export default CheckBox;
