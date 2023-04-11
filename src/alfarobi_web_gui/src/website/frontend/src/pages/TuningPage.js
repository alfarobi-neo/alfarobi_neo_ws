import React from "react";
import AppBar from "../components/AppBar";

function TuningPage() {
  return (
    <div className="flex flex-col h-screen bg-[#898989]">
      <AppBar />
      <div className="flex flex-col h-screen items-center justify-center">
        <p className="text-3xl text-white">
          Open the menu button to start tuning
        </p>
      </div>
    </div>
  );
}

export default TuningPage;
