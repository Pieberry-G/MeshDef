import os
import base64
import requests
from io import BytesIO
from typing import List, Union, Optional
from string import Template
import traceback
import json

def remove_trailing_comments(input_string):
    # Split the input string into lines
    lines = input_string.split('\n')

    # Process each line to remove comments
    processed_lines = []
    for line in lines:
        comment_index = line.find('//')
        if comment_index != -1:
            # Remove the comment
            line = line[:comment_index]
        processed_lines.append(line.strip())

    # Join the processed lines back into a single string
    return """{}""".format('\n'.join(processed_lines))

def parse_json_string(res, verbose=False):
    if '```' in res:
        try:
            res_clean = res

            if '```json' in res:
                res_clean = res_clean.split('```')[1].split('json')[1]
            elif '```JSON' in res:
                res_clean = res_clean.split('```')[1].split('JSON')[1]
            elif '```' in res:
                res_clean = res_clean.split('```')[1]
            else:
                print('Invalid response: ')
                print(res)

        except Exception:
            print(traceback.format_exc())
            print('Invalid response: ')
            print(res)
            return None
    else:
        res_clean = res

    try:
        res_filtered = remove_trailing_comments(res_clean)
        # 尝试解析JSON，如果失败则进行额外处理
        try:
            object_info = json.loads(res_filtered)
        except json.decoder.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
            # 检查是否缺少双引号的属性名或使用了单引号
            if "Expecting property name enclosed in double quotes" in str(e) or "Expecting '\"'" in str(e):
                # 尝试修复没有双引号的属性名
                import re
                # 查找没有引号的属性名并添加双引号
                fixed_json = re.sub(r'(\n\s*)([a-zA-Z0-9_]+)(\s*:)', r'\1"\2"\3', res_filtered)
                # 替换单引号为双引号
                fixed_json = fixed_json.replace("'", "\"")
                try:
                    object_info = json.loads(fixed_json)
                    print("已修复JSON格式并成功解析")
                except Exception as e2:
                    print(f"修复后仍然无法解析JSON: {e2}")
                    print("修复后的JSON内容:")
                    print(fixed_json)
                    return None
            else:
                return None

        # if verbose:
        #     print_object_info(object_info)

        return object_info

    except Exception:
        print(traceback.format_exc())
        print('The original response: ')
        print(res)
        print('Invalid cleaned response: ')
        print(res_clean)
        return None

def write_to_json(json_result, save_result_path, instruction):

    json_result['Ins'] = instruction
    json_result['Hand Action'] = json_result['Hand Action'].capitalize()
    json_result['Part'] = json_result['Part'].capitalize()
    with open(save_result_path, 'w') as j:
        json.dump(json_result, j, indent=4)
    j.close()


class OpenAIClient:
    """A client for interacting with the OpenAI API."""

    DEFAULT_LLM_MODEL_NAME = 'gpt-4'
    # DEFAULT_VLM_MODEL_NAME = 'gpt-4o-2024-11-20'
    # DEFAULT_VLM_MODEL_NAME = "gemini-2.0-flash-thinking-exp"
    # DEFAULT_VLM_MODEL_NAME = 'gpt-4.5-preview-2025-02-27'
    # DEFAULT_VLM_MODEL_NAME = 'gpt-4.1-mini'
    DEFAULT_VLM_MODEL_NAME = 'o4-mini'
    DEFAULT_API_URL = 'https://api.openai.com/v1/chat/completions'

    def __init__(self,
                 api_key: Optional[str] = None,
                 url: Optional[str] = None):
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("API key must be provided or set in the environment.")
        self.headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}'
        }
        self.url = url or self.DEFAULT_API_URL


    @staticmethod
    def encode_image_from_file(image_path: str) -> str:
        """Encode an image from a file path to base64 format."""
        with open(image_path, 'rb') as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    @staticmethod
    def encode_image_from_pil(image) -> str:
        """Encode a PIL image to base64 format."""
        buffered = BytesIO()
        image.save(buffered, format='JPEG')
        return base64.b64encode(buffered.getvalue()).decode('utf-8')

    @staticmethod
    def prepare_text_content(messages: Union[str, List[str]]) -> List[dict]:
        """Prepare text content for the API request."""
        if isinstance(messages, str):
            messages = [messages]
        return [{'type': 'text', 'text': message} for message in messages]

    def prepare_image_content(self, images: Union[str, List], local_image: bool) -> List[dict]:
        """Prepare image content for the API request."""
        if isinstance(images, str):
            images = [images]

        image_contents = []
        for image in images:
            if local_image:
                base64_image = self.encode_image_from_file(image)
            else:
                base64_image = self.encode_image_from_pil(image)

            image_contents.append({
                'type': 'image_url',
                'image_url': {
                    'url': f'data:image/jpeg;base64,{base64_image}'
                },
                # "detail": "high"
            })

        return image_contents

    def prepare_payload(self, messages: Union[str, List[str]],
                        images: Union[str, List],
                        meta_prompt: str,
                        model_name: str,
                        local_image: bool,
                        example_images: Optional[List] = None,
                        example_responses: Optional[List[str]] = None) -> dict:
        """Prepare the payload for the API request."""
        user_content = self.prepare_text_content(messages)

        if example_images and example_responses:
            for example_image, example_response in zip(example_images, example_responses):
                user_content.extend(self.prepare_image_content([example_image], local_image))
                user_content.append({'type': 'text', 'text': example_response})

        user_content.extend(self.prepare_image_content(images, local_image))

        if model_name == "o4-mini":
            return {
                'model': model_name,
                'messages': [
                {'role': 'system', 'content': meta_prompt},
                {'role': 'user', 'content': user_content}
            ],
            }
        else:
            return {
                'model': model_name,
                'messages': [
                {'role': 'system', 'content': meta_prompt},
                {'role': 'user', 'content': user_content}
            ],
                # 'max_tokens': 800,
                'temperature': 0.3,
            }

    def request_gpt(self, message: Union[str, List[str]],
                    images: Union[str, List],
                    meta_prompt: str = '',
                    model_name: Optional[str] = None,
                    local_image: bool = False,
                    example_images: Optional[List] = None,
                    example_responses: Optional[List[str]] = None
                    ) -> str:
        """Send a request to the GPT model."""
        if model_name is None:
            model_name = self.DEFAULT_LLM_MODEL_NAME if not images else self.DEFAULT_VLM_MODEL_NAME

        payload = self.prepare_payload(
            messages=message,
            images=images,
            meta_prompt=meta_prompt,
            model_name=model_name,
            local_image=local_image,
            example_images=example_images,
            example_responses=example_responses
        )

        response = requests.post(
            url=self.url,
            headers=self.headers,
            json=payload
        )

        return self.handle_response(response)

    def request_gpt_incontext(self, message: Union[str, List[str]],
                              images: Union[str, List],
                              meta_prompt: str = '',
                              example_images: Optional[List] = None,
                              example_responses: Optional[List[str]] = None,
                              model_name: Optional[str] = None,
                              local_image: bool = False) -> str:
        """Send a request to the GPT model with in-context examples."""
        if model_name is None:
            model_name = self.DEFAULT_LLM_MODEL_NAME if not images else self.DEFAULT_VLM_MODEL_NAME

        payload = self.prepare_payload(
            messages=message,
            images=images,
            meta_prompt=meta_prompt,
            model_name=model_name,
            local_image=local_image,
            example_images=example_images,
            example_responses=example_responses
        )

        response = requests.post(
            url=self.url,
            headers=self.headers,
            json=payload
        )

        return self.handle_response(response)

    @staticmethod
    def handle_response(response: requests.Response) -> str:
        """Handle the API response, return the content or raise an error."""
        try:
            response.raise_for_status()
            return response.json()['choices'][0]['message']['content']
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
            return f"Error: {e}"
        except (KeyError, IndexError) as e:
            print(f"Invalid response format: {response.text}")
            return f"Error: Invalid response format"

def load_prompts(prompt_dir):
    """Load prompts from files.
    """
    prompts = dict()
    for filename in os.listdir(prompt_dir):
        path = os.path.join(prompt_dir, filename)
        if os.path.isfile(path) and path[-4:] == '.txt':
            with open(path, 'r') as f:
                value = f.read()
            key = filename[:-4]
            prompts[key] = value
    return prompts

if __name__ == '__main__':
    # Example usage:
    import json
    from PIL import Image
    image = Image.open("assets/images/color_points.png").convert('RGB')
    meta_prompt = load_prompts("assets/prompt")["handle_selection_color"]
    text_prompt = "Lift the cow's horns."
    # text_prompt = "Move the cow's legs."

    # meta_prompt = "Describe image grid(s) occupied by the bottle."
    # 聚合AI
    api_key = 'sk-53YLmKSBBNHO6mQXhZWXxDkcBzTztpLWZhi8u9PlN2Ci6Lt0'
    url = 'https://api.bianxie.ai/v1/chat/completions'
    client = OpenAIClient(api_key, url)
    response = client.request_gpt(
        str(text_prompt), 
        [image], 
        meta_prompt=(meta_prompt), 
        model_name="gemini-3-pro-preview", 
        local_image=False)
    result = parse_json_string(response)
    print(response)