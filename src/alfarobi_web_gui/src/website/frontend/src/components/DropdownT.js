import React from "react";
import Popper from "popper.js";
import CheckBox from "./CheckBox";

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
              <CheckBox ros={ros} joint={data} />
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
