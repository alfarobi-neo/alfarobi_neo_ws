import React from "react";
import CardComponent from "./CardComponent";

function InputCard({ robot, titles, init_joint }) {
  // console.log(robot);
  return (
    <div className="bg-primary_bg mb-1 p-2 rounded-lg">
      <div className="flex flex-row text-center text-white">
        <p className="w-[7vw] text-[12px]">joint ID</p>
        <p className="w-[7vw] text-[12px]">Last taken value</p>
        <p className="w-[7vw] text-[12px]">Edited value</p>
        <p className="w-[7vw] text-[12px]">Saved value</p>
      </div>
      {titles.map((data, index) => (
        <CardComponent key={index} robot={robot} init_joint={init_joint}>
          {data}
        </CardComponent>
      ))}
    </div>
  );
}

export default InputCard;

//particle filtering
