import importlib.util
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
MODULE_PATH = CURRENT_DIR / "07_cloud_chat.py"

spec = importlib.util.spec_from_file_location("cloud_chat_module", MODULE_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Cannot load {MODULE_PATH}")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

cloud_chat = module.cloud_chat
speak_text = module.speak_text
speak_text_reliable = module.speak_text_reliable
