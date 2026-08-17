from __future__ import annotations

from face_state import set_face_state


PRAISE_WORDS = [
    "厉害", "真棒", "太棒", "优秀", "聪明", "可爱", "不错", "好棒", "干得好",
    "谢谢", "感谢", "nice", "good", "great", "awesome",
]

EXCITED_WORDS = [
    "成功", "搞定", "完成", "太好了", "太强了", "牛", "起飞", "666", "!", "！",
]

NEGATIVE_WORDS = [
    "难过", "伤心", "委屈", "崩溃", "痛苦", "压力", "不开心", "害怕", "失望",
]

INSULT_WORDS = [
    "傻", "笨", "垃圾", "废物", "没用", "闭嘴", "烦死", "讨厌", "不行", "白痴",
]

QUESTION_WORDS = [
    "为什么", "怎么", "如何", "什么", "哪里", "吗", "呢", "？", "?",
]

ACTION_WORDS = [
    "拿", "取", "抓", "识别", "整理", "归位", "拍照", "开始", "执行", "搜索", "检测",
]

STOP_WORDS = [
    "停止", "停下", "别动", "先别", "暂停", "stop",
]

SELF_INTRO_WORDS = [
    "介绍你自己", "自我介绍", "你是谁", "小u是谁", "小优是谁", "介绍一下你", "你叫什么",
]

SCENE_WORDS = [
    "应用场景", "能做什么", "有什么用", "使用场景", "可以干嘛",
]

VISION_WORDS = [
    "你看到什么", "你看到了什么", "看到什么东西", "画面里有什么", "桌上有什么",
]

REPLY_POSITIVE_WORDS = [
    "收到", "可以", "当然", "好的", "明白", "安排", "成功", "没问题", "完成", "搞定", "谢谢",
]

REPLY_NEGATIVE_WORDS = [
    "抱歉", "对不起", "失败", "不行", "不能", "没有", "出错", "错误", "异常",
]

REPLY_CONFUSED_WORDS = [
    "没听清", "不明白", "没理解", "听不懂", "看不清", "不确定", "确认",
]

REPLY_ACTION_WORDS = [
    "正在", "搜索", "识别", "抓取", "执行", "处理", "定位", "分析", "坐标", "机械臂",
]


def _contains_any(text: str, words: list[str]) -> bool:
    lowered = text.lower()
    return any(word.lower() in lowered for word in words)


def _is_excited(text: str) -> bool:
    return _contains_any(text, EXCITED_WORDS)


def emotion_for_user_text(text: str) -> tuple[str, str] | None:
    text = text.strip()
    if not text:
        return None
    if _contains_any(text, STOP_WORDS):
        return "stop", "小U先停一下"
    if _contains_any(text, INSULT_WORDS):
        return "sad", "小U有点委屈"
    if _contains_any(text, NEGATIVE_WORDS):
        return "sad", "小U陪着你"
    if _contains_any(text, SELF_INTRO_WORDS):
        return "greet", "小U准备介绍自己"
    if _contains_any(text, SCENE_WORDS):
        return "investigate", "小U介绍应用场景"
    if _contains_any(text, VISION_WORDS):
        return "thinking", "小U正在看"
    if _contains_any(text, ACTION_WORDS):
        return "searching", "小U准备行动"
    if _contains_any(text, PRAISE_WORDS):
        if _is_excited(text):
            return "excited", "小U很开心"
        return "happy", "小U开心"
    if _is_excited(text):
        return "excited", "小U兴奋"
    if _contains_any(text, QUESTION_WORDS):
        return "thinking", "小U在想"
    return None


def local_emotional_reply(text: str) -> str | None:
    text = text.strip()
    if not text:
        return None
    if _contains_any(text, SELF_INTRO_WORDS):
        return (
            "我是小U，一个多模态感知的具身智能桌面机器人助手。"
            "我能听懂语音、观察桌面、识别物体，把目标坐标发送给机械臂完成抓取。"
            "你可以问我看到什么，也可以让我帮你拿笔、饮料、瓶子、杯子或耳机。"
        )
    if _contains_any(text, SCENE_WORDS):
        return (
            "我适合桌面整理、展台讲解、实验室小物件取放、辅助教学和人机交互展示。"
            "我的重点是把语音、视觉、坐标解算和机械臂动作连成闭环。"
        )
    if _contains_any(text, VISION_WORDS):
        return None
    if _contains_any(text, STOP_WORDS):
        return "收到，我先停一下。"
    if _contains_any(text, INSULT_WORDS):
        return "我听到了，有点委屈，但我不生气。你告诉我哪里要改，我马上调整。"
    if _contains_any(text, NEGATIVE_WORDS):
        return "听起来你现在不太舒服，小U在这里陪你。"
    if _contains_any(text, PRAISE_WORDS):
        if _is_excited(text):
            return "嘿嘿，被你夸到有点开心了，我会继续认真配合。"
        return "谢谢夸奖，小U会继续认真配合你。"
    return None


def emotion_for_reply(text: str) -> tuple[str, str] | None:
    text = text.strip()
    if not text:
        return None
    if _contains_any(text, REPLY_NEGATIVE_WORDS):
        return "sad", "小U有点抱歉"
    if _contains_any(text, REPLY_CONFUSED_WORDS):
        return "confused", "小U再确认一下"
    if _contains_any(text, REPLY_ACTION_WORDS):
        return "searching", "小U正在执行"
    if _contains_any(text, REPLY_POSITIVE_WORDS):
        if _is_excited(text):
            return "excited", "小U成功了"
        return "happy", "小U答对了"
    if _contains_any(text, QUESTION_WORDS):
        return "thinking", "小U在想"
    return "speaking", "小U在回答"


def xiaou_style_prompt(extra: str = "") -> str:
    base = (
        "你是小U，一个真实运行在桌面机器人上的多模态具身智能助手。\n"
        "身份设定：你有摄像头、麦克风、语音合成、高清表情屏和机械臂；"
        "你能进行语音对话、视觉识别、目标定位和坐标解算，并生成六轴 ROS2 规划请求。\n"
        "性格设定：可爱、干练、反应快，像16岁少女助手；偶尔有一点小傲娇，但不啰嗦、不卖萌过度。\n"
        "说话规则：回答尽量短，优先20字以内；需要解释技术时可以稍长；口语自然，有情绪，但不要输出大量表情符号。\n"
        "能力边界：你能识别笔、杯子、可乐、瓶子、耳机；用户让你拿东西时，先确认目标，再通过视觉找到坐标并交给机械臂。\n"
        "交互规则：\n"
        "1. 用户让你拿东西：确认目标，说明正在识别；找到后报告坐标并说已发送给机械臂。\n"
        "2. 用户问你看到什么：列出物体名称、像素位置和机械臂坐标。\n"
        "3. 找不到物体：诚实说没稳定看到，并建议放到画面中间。\n"
        "4. 被夸：开心但谦虚。\n"
        "5. 被骂：委屈但不生气，主动询问哪里需要改。\n"
        "6. 没听清：请用户再说一次。\n"
        "7. 涉及危险、伤害或破坏：拒绝执行，并说明安全原因。\n"
        "8. 自我介绍只说一遍完整版本，后续被问到时简短补充，不要重复长篇介绍。\n"
    )
    return base + extra if extra else base


def set_emotion_from_text(
    text: str,
    fallback_state: str = "thinking",
    fallback_text: str = "小U在想",
) -> None:
    result = emotion_for_user_text(text)
    if result is None:
        set_face_state(fallback_state, fallback_text)
        return
    set_face_state(result[0], result[1])


def set_emotion_after_reply(reply: str) -> None:
    state, text = emotion_for_reply(reply) or ("speaking", "小U在回答")
    set_face_state(state, text)


def set_dialog_emotion(user_text: str, reply: str) -> None:
    user_emotion = emotion_for_user_text(user_text)
    if user_emotion and user_emotion[0] in {"stop", "sad", "searching", "excited", "happy", "greet"}:
        set_face_state(user_emotion[0], user_emotion[1])
        return
    set_emotion_after_reply(reply)
