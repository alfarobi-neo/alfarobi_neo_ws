import React from "react";
import Sidebar from "./Sidebar";
import logo from "../assets/alfarobi_logo_head.png";

function AppBar() {
  return (
    <div className="sticky top-0 flex flex-row bg-[#232323] items-center min-h-[40px] py-[2px]">
      <Sidebar />
      <div className="flex flex-row mx-auto items-center">
        <img
          className="object-scale-down max-h-[30px] mr-3"
          src={logo}
          alt="logo"
        />
        <p className="text-2xl text-white">
          Alfarobi Web GUI (per-robot tuning)
        </p>
      </div>
    </div>
  );
}

export default AppBar;
