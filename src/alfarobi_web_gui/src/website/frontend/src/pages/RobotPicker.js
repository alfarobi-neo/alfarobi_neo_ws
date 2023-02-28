import React from "react";
import RobotPickerButton from "../components/RobotPickerButton";

function RobotPicker() {
  return (
    <div className="flex flex-col items-center bg-primary_bg h-screen">
      <p className="text-white text-[3.5vw] my-12">Please choose the robot</p>
      <div className="flex">
        <RobotPickerButton>Abi</RobotPickerButton>
        <RobotPickerButton>Alfa</RobotPickerButton>
        <RobotPickerButton>Robi</RobotPickerButton>
      </div>
    </div>
  );
}

export default RobotPicker;
