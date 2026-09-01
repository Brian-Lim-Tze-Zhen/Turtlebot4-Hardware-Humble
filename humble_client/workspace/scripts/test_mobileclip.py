#!/usr/bin/env python3
import open_clip
import torch
from PIL import Image

# Same prompt pair as group_formation_detector.py (validated minimal-contrast
# negation pair — index 0 MUST stay the "conversation" answer).
PROMPTS = [
    "two people facing each other",
    "two people not facing each other",
]

VLM_MIN_CONFIDENCE = 0.55


def classify_facing(image_path):
    print("Loading MobileCLIP-S1...")
    model, _, preprocess = open_clip.create_model_and_transforms(
        "MobileCLIP-S1", pretrained="datacompdr"
    )
    model.eval()
    tokenizer = open_clip.get_tokenizer("MobileCLIP-S1")

    text_tokens = tokenizer(PROMPTS)
    with torch.no_grad():
        text_features = model.encode_text(text_tokens)
        text_features = text_features / text_features.norm(dim=-1, keepdim=True)

    image = Image.open(image_path).convert("RGB")
    image_input = preprocess(image).unsqueeze(0)

    with torch.no_grad():
        image_features = model.encode_image(image_input)
        image_features = image_features / image_features.norm(dim=-1, keepdim=True)
        similarities = (100.0 * image_features @ text_features.T).softmax(dim=-1)

    print("\n--- MobileCLIP Output ---")
    for i, prompt in enumerate(PROMPTS):
        print(f"  [{i}] '{prompt}' = {float(similarities[0, i]):.3f}")

    best_idx = int(similarities.argmax())
    best_score = float(similarities[0, best_idx])
    print(f"\nBest match: '{PROMPTS[best_idx]}' ({best_score:.3f})")

    if best_score < VLM_MIN_CONFIDENCE:
        print(f"Below VLM_MIN_CONFIDENCE ({VLM_MIN_CONFIDENCE}) -> inconclusive (None)")
        return None

    result = best_idx == 0
    print(f"Confirmed facing each other: {result}")
    return result


if __name__ == "__main__":
    import sys
    img_path = sys.argv[1] if len(sys.argv) > 1 else "legs.jpg"
    classify_facing(img_path)
