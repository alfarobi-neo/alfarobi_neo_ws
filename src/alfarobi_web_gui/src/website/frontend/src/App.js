import "./App.css";
import LandingPage from "./pages/LandingPage";
import RobotPicker from "./pages/RobotPicker";
import TuningPage from "./pages/TuningPage";
import InitialPoses from "./pages/InitialPoses";
import SequencerTest from "./pages/SequencerTest";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";

function App() {
  return (
    <div className="App">
      <Router>
        <Routes>
          <Route exact path="/" element={<LandingPage />} />
          <Route exact path="/sequencer/:type" element={<SequencerTest />} />
          <Route path="/pick" element={<RobotPicker />} />
          <Route path="/tune/:robot" element={<TuningPage />} />
          <Route path="/init/:robot" element={<InitialPoses />} />
        </Routes>
      </Router>
    </div>
  );
}

export default App;
