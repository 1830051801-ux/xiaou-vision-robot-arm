param(
    [string]$PythonPath = "D:\EmbodiedAI\mujoco-venv\Scripts\python.exe",
    [int]$TrainCount = 4096,
    [int]$TestCount = 640,
    [int]$Epochs = 400,
    [string]$TaskSuite = "transfer,sort_zone_a,sort_zone_b,inspection_scan,precision_insert",
    [int]$BeliefViews = 5,
    [int]$BeliefCandidates = 4,
    [double]$BeliefOutlierProbability = 0.20
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path -LiteralPath $PythonPath)) {
    throw "Python runtime not found: $PythonPath. Use a Python environment with PyTorch, NumPy, and Matplotlib."
}

Set-Location -LiteralPath $root
$runtime = Join-Path $root "runtime\constraint_diffusion"
$checkpoint = Join-Path $runtime "process_graph_multitask_action_chunk_transformer_policy.pt"
$train = Join-Path $runtime "multitask_train.npz"
$test = Join-Path $runtime "multitask_test.npz"
$shift = Join-Path $runtime "multitask_test_shift.npz"
$nominalReport = Join-Path $runtime "process_graph_belief_evaluation_nominal.json"
$shiftReport = Join-Path $runtime "process_graph_belief_evaluation_shift.json"

& $PythonPath tools\constraint_diffusion_twin.py generate --output $train --count $TrainCount --seed 20260815 --tasks $TaskSuite --domain-randomization
& $PythonPath tools\constraint_diffusion_twin.py generate --output $test --count $TestCount --seed 20260816 --tasks $TaskSuite
& $PythonPath tools\constraint_diffusion_twin.py generate --output $shift --count $TestCount --seed 20260817 --xy-sigma-m 0.008 --z-sigma-m 0.004 --yaw-sigma-deg 5.0 --tasks $TaskSuite
& $PythonPath tools\constraint_diffusion_twin.py train --dataset $train --checkpoint $checkpoint --epochs $Epochs --batch-size 64 --hidden-dim 128 --diffusion-steps 16 --seed 20260815
& $PythonPath tools\constraint_diffusion_twin.py evaluate --checkpoint $checkpoint --dataset $test --output $nominalReport --samples-per-context 3 --abstain-dispersion-rad 0.45 --seed 20260815 --counterfactual-rollouts 4 --minimum-counterfactual-safe-fraction 0.75 --counterfactual-noise-fraction 0.50 --belief-views $BeliefViews --belief-candidate-count $BeliefCandidates --belief-outlier-probability $BeliefOutlierProbability --belief-minimum-safe-fraction 0.75
& $PythonPath tools\constraint_diffusion_twin.py evaluate --checkpoint $checkpoint --dataset $shift --output $shiftReport --samples-per-context 3 --abstain-dispersion-rad 0.45 --seed 20260815 --counterfactual-rollouts 4 --minimum-counterfactual-safe-fraction 0.75 --counterfactual-noise-fraction 0.50 --belief-views $BeliefViews --belief-candidate-count $BeliefCandidates --belief-outlier-probability $BeliefOutlierProbability --belief-minimum-safe-fraction 0.75
& $PythonPath tools\render_multitask_factory_cell_dashboard.py --checkpoint $checkpoint --nominal $nominalReport --shift $shiftReport --dataset $test --output assets\multitask_factory_cell_dashboard.png
