---
layout: page
title: 日记本
permalink: /diaries/
---

<section class="diaries-collection">
  <div class="diary-intro">
    这里是团队成员的学习日志,记录日常学习过程、技术探索和项目实践。
  </div>

  <div class="diary-books-grid">
    {% for author in site.data.authors %}
      {% assign author_key = author[0] %}
      {% assign author_data = author[1] %}
      {% unless author_data.non_tech %}
        {% assign author_diaries = site.diaries | where: "author", author_key %}

        {% if author_diaries.size > 0 %}
          <article class="diary-book-card">
          <div class="diary-book-header">
            <h2>
              <a href="/diaries/{{ author_key }}/">
                {{ author_data.display_name }}的日记本
              </a>
            </h2>
            <p class="author-bio">{{ author_data.bio }}</p>
            <p class="diary-count">{{ author_diaries.size }} 篇日志</p>
          </div>
          <a href="/diaries/{{ author_key }}/" class="view-diaries">查看日记 →</a>
        </article>
      {% endif %}
    {% endunless %}
    {% endfor %}
  </div>
</section>

<style>
  .diary-intro {
    font-size: 1.1rem;
    line-height: 1.8;
    margin-bottom: 4rem;
    padding: 2rem;
    border-left: 8px solid #000;
    background: #f5f5f5;
  }

  .diary-books-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 2rem;
  }

  .diary-book-card {
    border: 4px solid #000;
    padding: 2rem;
    background: #fff;
    transition: all 0.3s;
    display: flex;
    flex-direction: column;
  }

  .diary-book-card:hover {
    transform: translate(-6px, -6px);
    box-shadow: 12px 12px 0 #000;
  }

  .diary-book-header {
    flex: 1;
  }

  .diary-book-header h2 {
    font-size: 1.5rem;
    margin: 0 0 0.8rem 0;
    border: none;
    padding-left: 0 !important;
  }

  .diary-book-header h2 a {
    color: #000;
    text-decoration: none;
    border-bottom: 3px solid transparent;
    transition: all 0.2s;
  }

  .diary-book-header h2 a:hover {
    border-bottom-color: #000;
    background: #000;
    color: #fff !important;
    padding: 0 0.3rem;
  }

  .author-bio {
    font-size: 0.9rem;
    line-height: 1.5;
    opacity: 0.75;
    margin-bottom: 0.8rem;
  }

  .diary-count {
    font-size: 0.8rem;
    font-weight: 700;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    opacity: 0.6;
    margin-bottom: 1.2rem;
  }

  .view-diaries {
    display: inline;
    align-self: flex-start;
    font-weight: 700;
    font-size: 0.85rem;
    color: #000 !important;
    text-decoration: none !important;
    border-bottom: 2px solid transparent !important;
    line-height: 1.4;
    transition: all 0.2s;
  }

  .view-diaries:hover {
    background: #000;
    color: #fff !important;
    padding: 0 0.2rem;
    border-bottom: 2px solid #000 !important;
  }

  @media (max-width: 768px) {
    .diary-books-grid {
      grid-template-columns: 1fr;
    }

    .diary-book-card {
      padding: 1.5rem;
    }

    .diary-book-header h2 {
      font-size: 1.6rem;
    }
  }
</style>
