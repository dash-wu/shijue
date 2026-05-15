#!/usr/bin/env python3
import os
import re
import sys

# 避免系统 SOCKS 代理导致 httpx 报错: Unknown scheme for proxy URL 'socks://...'
for _k in list(os.environ.keys()):
    if _k.lower() in ("all_proxy", "http_proxy", "https_proxy") or _k.lower().endswith(
        "_proxy"
    ):
        _v = os.environ.get(_k, "")
        if "socks" in _v.lower():
            del os.environ[_k]

import httpx
from openai import OpenAI

def _load_api_key():
    p = os.path.join(os.path.dirname(__file__), "moonshot_api_key.txt")
    if os.path.isfile(p):
        with open(p, encoding="utf-8") as fp:
            line = fp.readline().strip()
            if line:
                return line
    return os.environ.get("MOONSHOT_API_KEY", "").strip()


api_key = _load_api_key()
if not api_key:
    raise RuntimeError(
        "未配置 Moonshot API Key：请在 scripts/moonshot_api_key.txt 写入一行密钥，"
        "或设置环境变量 MOONSHOT_API_KEY"
    )
base_url = "https://api.moonshot.cn/v1"


class KimiClient(OpenAI):
    """Moonshot Kimi（与语音/交互共用配置）"""

    def __init__(self):
        super().__init__(
            api_key=api_key,
            base_url=base_url,
            http_client=httpx.Client(trust_env=False),
        )
        self.model = "moonshot-v1-8k"
        self.system_role_content = (
            "你是Kimi,由Moonshot AI提供的人工智能助手, "
            "我们将会叫你的小名“小月”,你不会在你的回答中提及你的小名,你更擅长中文和"
            "英文的对话. "
            "你会为用户提供安全,有帮助,准确的回答. "
            "同时,你会拒绝一切涉及恐怖主义,种族歧视,黄色暴力等问题的回答"
        )

    def get_system_role_prompt(self):
        return {"role": "system", "content": self.system_role_content}

    def user_prompt(self, user_prompt):
        return {"role": "user", "content": user_prompt}

    def query(self, user_prompt, max_tokens=None):
        sys_content = self.system_role_content
        if max_tokens is not None:
            sys_content += " 当前为语音对话，请先用一两句话直接回答要点，避免长段落。"
        user_message = [
            {"role": "system", "content": sys_content},
            self.user_prompt(user_prompt),
        ]
        kwargs = {
            "model": self.model,
            "messages": user_message,
            "temperature": 0.1,
            "stream": False,
        }
        # 语音模式外部传入较短 max_tokens，可明显缩短 API + 后续合成耗时
        if max_tokens is not None:
            kwargs["max_tokens"] = max_tokens
        completion = self.chat.completions.create(**kwargs)
        return completion.choices[0].message.content


def run_interactive():
    """与教材截图一致：终端问答循环（无需 roscore）"""
    try:
        llm = KimiClient()
    except Exception as e:
        print("大模型初始化失败:", e)
        return

    print("大模型初始化成功：")
    print("模型BaseURL:", base_url)
    print("模型版本:", llm.model)
    print()

    while True:
        try:
            q = input("输入你的问题（或输入 '退出' 结束程序）：")
        except EOFError:
            print()
            break
        if q.strip() in ("退出", "exit", "quit", "q"):
            break
        if not q.strip():
            continue
        try:
            chat_response = llm.query(q)
            indented = "\n".join(f"\t{line}" for line in chat_response.splitlines())
            print(f"LLM的返回结果：\n\n'''\n{indented}\n'''")
        except Exception as e:
            if "rate_limit_reached" in str(e):
                print("请求超限")
            else:
                print("出错啦:", e)
        print()


def run_ros():
    """订阅 /speech/result，与语音识别联调：rosrun my_class_pkg llm_chat.py --ros"""
    import rospy
    import rosgraph
    from rosgraph.masterapi import Failure as RosgraphFailure
    from rosgraph.masterapi import Error as RosgraphError
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Empty

    class LLM(KimiClient):
        def __init__(self):
            super().__init__()
            rospy.init_node("robot_voice_llm_node", anonymous=True)
            rospy.Subscriber("/speech/result", String, self.speech_result_callback)
            # upros_chat：/tts_playing 先于 /talk，让识别端尽早停送麦；/talk 为待播正文
            self._tts_playing_pub = rospy.Publisher("/tts_playing", Bool, queue_size=2)
            self._tts_pub = rospy.Publisher("/talk", String, queue_size=2)
            rospy.sleep(0.15)

        @staticmethod
        def _text_for_speaker(text):
            """与终端打印同一套正文，只去 Markdown/代码块；不强压成一行、不大量截断，避免和听见的不一致。"""
            if not text:
                return ""
            s = re.sub(r"```[\s\S]*?```", " ", text)
            s = re.sub(r"\*\*([^*]+)\*\*", r"\1", s)
            s = re.sub(r"#{1,6}\s*", "", s)
            s = re.sub(r"[ \t]*\n[ \t]*", "，", s)
            s = re.sub(r"[ \t]+", " ", s).strip()
            if len(s) > 1200:
                s = s[:1200] + "……"
            return s

        @staticmethod
        def _worth_send_to_llm(text):
            t = (text or "").strip()
            if len(t) < 3:
                return False
            noise = {"嗯", "啊", "呃", "哦", "噢", "哈", "哎"}
            if len(t) <= 2 and all((c in noise or c in "，。！？") for c in t):
                return False
            return True

        def _ready_next_utterance(self):
            """upros_chat 识别发布一次后会关闸，须调 /start_recognize 才能说下一句。
            不用 rospy.wait_for_service：会在终端刷黄色 WARN；若服务未就绪则静默跳过。"""
            try:
                rosgraph.Master(rospy.get_name()).lookupService("/start_recognize")
            except (RosgraphFailure, RosgraphError, OSError, ConnectionRefusedError):
                return
            try:
                rospy.ServiceProxy("/start_recognize", Empty)()
            except rospy.ServiceException:
                pass

        def speech_result_callback(self, msg):
            result = msg.data
            print("speech [{}]".format(result))
            if result and not self._worth_send_to_llm(result):
                print(
                    "（识别结果过短或为语气词，已跳过本次大模型；请说完整问句。）"
                )
                self._ready_next_utterance()
                return
            if result:
                try:
                    # 语音场景限制回复长度，整体延迟可下降数秒（接口+合成均更短）
                    chat_response = self.query(result, max_tokens=320)
                    indented_response = "\n".join(
                        f"\t{line}" for line in chat_response.splitlines()
                    )
                    print(f"LLM的返回结果: \n\n'''\n{indented_response}\n'''")
                    spoken = self._text_for_speaker(chat_response)
                    if spoken:
                        self._tts_playing_pub.publish(Bool(True))
                        self._tts_pub.publish(String(data=spoken))
                        rospy.loginfo("已发送到扬声器合成 (/talk)，长度=%d", len(spoken))
                except Exception as e:
                    if "rate_limit_reached" in str(e):
                        print("请求超限")
                    else:
                        print("出错啦")
                finally:
                    self._ready_next_utterance()

    try:
        llm = LLM()
        rospy.spin()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")


if __name__ == "__main__":
    if "--ros" in sys.argv:
        run_ros()
    else:
        run_interactive()
