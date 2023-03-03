import React from "react";
import Popper from "popper.js";

function DropdownBM({ color }) {
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
          className="text-black w-[21vw] pl-2 text-left text-sm rounded shadow hover:bg-white outline-none focus:outline-none bg-secondary_bg"
          style={{ transition: "all .15s ease" }}
          type="button"
          ref={btnDropdownRef}
          onClick={() => {
            dropdownPopoverShow
              ? closeDropdownPopover()
              : openDropdownPopover();
          }}
        >
          Ready
        </button>
        <div
          ref={popoverDropdownRef}
          className={
            (dropdownPopoverShow ? "block" : "hidden ") +
            "bg-white text-base w-[21vw] float-left py-2 list-none text-left rounded shadow-lg mt-1"
          }
          style={{ minWidth: "12rem" }}
        >
          <a
            href="#babi"
            className={
              "text-sm py-2 px-4 font-normal block whitespace-no-wrap bg-white text-black"
            }
            onClick={(e) => e.preventDefault()}
          >
            Babi
          </a>
          <a
            href="#anjing"
            className={
              "text-sm py-2 px-4 font-normal block whitespace-no-wrap bg-white text-black"
            }
            onClick={(e) => e.preventDefault()}
          >
            Anjing
          </a>
        </div>
      </div>
    </div>
  );
}

export default DropdownBM;
