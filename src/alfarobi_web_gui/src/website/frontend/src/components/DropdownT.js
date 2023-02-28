import React from "react";
import Popper from "popper.js";
import CheckBox from "./CheckBox";

function DropdownT({ color }) {
  const props = [
    "r_sho_roll",
    "r_sho_pitch",
    "r_el_pitch",
    "r_hip_roll",
    "r_hip_pitch",
    "r_hip_yaw",
    "r_knee_pitch",
    "r_ank_roll",
    "r_ank_pitch",
    "head_tilt",
    "head_pan",
    "l_sho_roll",
    "l_sho_pitch",
    "l_el_pitch",
    "l_hip_roll",
    "l_hip_pitch",
    "l_hip_yaw",
    "l_knee_pitch",
    "l_ank_roll",
    "l_ank_pitch",
  ];
  // dropdown props
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
              <CheckBox>{data}</CheckBox>
              {/* <a
                className={
                  "text-sm py-2 px-4 font-normal block whitespace-no-wrap bg-white text-black hover:bg-secondary_bg hover:cursor-pointer"
                }
                key={index}
              >
                {data}
              </a> */}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

export default DropdownT;
