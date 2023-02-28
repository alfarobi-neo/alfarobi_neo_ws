import React from "react";
import { useState } from "react";

function CheckBox(props) {
  const [checked, setChecked] = useState(false);

  const handleChange = () => {
    setChecked(!checked);
  };

  return (
    <label>
      <div className="flex flex-row text-sm pl-4 font-normal whitespace-no-wrap bg-white text-black">
        <input
          className="mx-2"
          type="checkbox"
          checked={checked}
          onChange={handleChange}
        />
        <p>{props.children}</p>
      </div>
    </label>
  );
}

export default CheckBox;
