# Manual Data Ingestion & Setup Guide

Since you have deployed the frontend to Vercel, you still need to ensure your **Backend** and **Database** are ready.

## 1. Why do I need the script?
The `ingest.py` script does physically read your markdown files, but it sends the data to **Qdrant Cloud**.
- **Frontend (Vercel)**: Displays data.
- **Backend**: Processes logic.
- **Qdrant (Cloud)**: Stores the "Brain" of the textbook.

Even if you deploy the frontend, the **Qdrant Database starts empty**. You must run the script **ONCE** from your computer to "teach" the AI the contents of your book. It uploads the data to the cloud, so your deployed Vercel app can access it.

## 2. How to "Manually" Ingest
"Manual" ingestion just means running the script yourself locally.

### Step-by-Step Commands
1. **Open Terminal** in your project folder.
2. **Navigate** to the inner `hackathon` folder:
   ```bash
   cd hackathon
   ```
3. **Install Dependencies** (if you haven't):
   ```bash
   pip install -r backend/requirements.txt
   ```
4. **Run Ingestion**:
   ```bash
   python backend/scripts/ingest.py
   ```
   *You will see output asking "Processing... Upserted batch..."*

## 3. Deployment Note
Your Vercel frontend currently points to `localhost` (in your code). For it to work online:
1. You must deploy this **FastAPI Backend** to a cloud provider (like **Render**, **Railway**, or **AWS**).
2. You must update your Frontend's API URL to point to that live Backend URL instead of `localhost:8000`.
