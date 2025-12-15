import os
import base64
import requests
from io import BytesIO
from typing import List, Union, Optional
from PIL import Image
import math

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

def merge_images(images_dir):
    image_extensions = {'.png', '.jpg', '.jpeg', '.bmp', '.gif', '.tiff', '.webp'}
    image_files = []
    
    for file in sorted(os.listdir(images_dir)):  # 排序保证顺序一致
        if any(file.lower().endswith(ext) for ext in image_extensions):
            image_files.append(os.path.join(images_dir, file))

    images = []
    for img_path in image_files:
        img = Image.open(img_path).convert('RGB')
        images.append(img)

    widths = [img.width for img in images]
    heights = [img.height for img in images]
    img_width = int(sum(widths) / len(widths))
    img_height = int(sum(heights) / len(heights))

    num_images = len(images)

    cols = int(math.ceil(math.sqrt(num_images)))
    rows = int(math.ceil(num_images / cols))
    while (rows - 1) * cols >= num_images:
        rows -= 1

    result_width = img_width * cols
    result_height = img_height * rows
    result_image = Image.new('RGB', (result_width, result_height), color='white')

    for index, img in enumerate(images):
        row = index // cols
        col = index % cols
        
        x_offset = col * img_width
        y_offset = row * img_height
        
        result_image.paste(img, (x_offset, y_offset))

    return images



def select_handles(images_dir):
    images = merge_images(images_dir)
    for img in images:
        img.save(images_dir + "../CandidateHandles_merged/output.png")
    # result_image.save(images_dir + "../CandidateHandles_merged/output.png")

    # Example usage:
    image = Image.open("../python/assets/images/2.png").convert('RGB')
    meta_prompt = load_prompts("../python/assets/prompt")["handle_selection_color"]
    text_prompt = "Lift the cow's horns."
    # text_prompt = "Move the cow's legs."

    # meta_prompt = "Describe image grid(s) occupied by the bottle."
    # 聚合AI
    api_key = 'sk-53YLmKSBBNHO6mQXhZWXxDkcBzTztpLWZhi8u9PlN2Ci6Lt0'
    url = 'https://api.bianxie.ai/v1/chat/completions'
    client = OpenAIClient(api_key, url)
    response = client.request_gpt(
        str(text_prompt), 
        images, 
        meta_prompt=(meta_prompt), 
        model_name="gemini-3-pro-preview", 
        local_image=False)
    
    return response