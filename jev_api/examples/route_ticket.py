"""TypeSafe 官方 quickstart：一次请求回答三种题型。"""

from typesafe_sdk import Choice, Noul, Score, TypeSafeClient


# 换成自己的官方 TypeSafe API key 即可运行。
API_KEY = "apikey_2180a45feea342e94466a7b4be0e8439285f_2da086b9287fa9932ad47f2067309b66de3d24440dbf189df165df8c54f28c3d"


def main():
    # state：待判断的材料，沿用官方示例的客服工单。
    ticket = "Our API integration started returning 500 errors on every request about 20 minutes ago, and we can't process any customer orders until this is fixed."

    with TypeSafeClient(
        api_key=API_KEY,
        base_url="https://api.typesafe.ai",
        model="jev-latest",
        timeout=30.0,
    ) as client:
        response = client.system_one(
            state=ticket,
            questions={
                # choice：从三个部门中选择一个。
                "department": Choice(
                    instructions="Which team should handle this",
                    criteria={
                        "billing": "Payment or subscription issues",
                        "technical": "Bugs or integration problems",
                        "sales": "Pricing or account questions",
                    },
                ),
                # score：按数组顺序定义等级，位置为 0、1、2。
                "frustration": Score(
                    instructions="How frustrated the customer appears",
                    criteria=[
                        "Calm, just stating facts",
                        "Frustrated but civil",
                        "Very angry, strong language",
                    ],
                ),
                # noul：返回这个命题为真的概率，范围 0～1。
                "is_urgent": Noul(
                    instructions="The message conveys urgency or time-sensitivity",
                ),
            },
        )

    print("department:", response.choices["department"].choice)
    print("frustration (0-2):", response.scores["frustration"].score)
    print("is_urgent (0-1):", response.nouls["is_urgent"].noul)
    print("\nFull response:")
    print(response.model_dump_json(indent=2))


if __name__ == "__main__":
    main()
