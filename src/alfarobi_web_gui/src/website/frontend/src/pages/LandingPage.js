import React from "react";
import LandingPageButton from "../components/LandingPageButton";
import logo from "../assets/alfarobi_logo_remake_horizontal_white_1.png";

function LandingPage() {
  return (
    <div className="bg-primary_bg h-screen">
      <div className="p-[3.2vw] italic text-white text-3xl">
        <p>Welcome to</p>
        <span className="text-6xl">Alfarobi Web GUI</span>
      </div>
      <div className="flex flex-col items-center mt-16">
        <p className=" italic text-white font-normal text-4xl">
          What do you want to do?
        </p>
        <div className="flex flex-wrap w-screen justify-center my-[4vh]">
          <LandingPageButton>per-robot tuning</LandingPageButton>
          <LandingPageButton>team-based tuning</LandingPageButton>
        </div>
      </div>
      <img
        className="object-scale-down max-h-14 fixed bottom-0 m-11"
        src={logo}
        alt="logo"
      />
    </div>
  );
}

export default LandingPage;
