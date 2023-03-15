import React from "react";
import { useState } from "react";
import ROSLIB from "roslib";

function CheckBox({ joint, ros }) {
  const [checked, setChecked] = useState(true);

  const handleChange = () => {
    setChecked(!checked);
  };

  var setTorque = new ROSLIB.Topic({
    ros: ros,
    name: "/torque",
    messageType: "alfarobi_web_gui/Torque",
  });

  var torque = new ROSLIB.Message({
    joint_name: joint,
    joint_state: checked,
  });

  return (
    <label>
      <div className="flex flex-row text-sm pl-4 font-normal whitespace-no-wrap bg-white text-black">
        <input
          className="mx-2"
          type="checkbox"
          checked={checked}
          onChange={() => {
            handleChange();
            torque.joint_state = !checked;
            setTorque.publish(torque);
          }}
        />
        <p>{joint}</p>
      </div>
    </label>
  );
}

export default CheckBox;
