---
title: Home
layout: base.njk
---

<div class="main-content">
  <h1 class="title">Movement Science Club</h1>
  <!-- A thin horizontal rile -->
  <hr>
  <p>Welcome to the resource page of the Movement Science Club of the Department of Bioengineering at the Christian Medical College Vellore, India.</p>
  <p>The club's repository contains various blogposts, resource links, and code that related to the different activities of the club.</p>

  <!-- Blogposts -->
  <h2 class="subtitle1">Blog Posts</h2>
  <table class="blog-table">
    <!-- <thead>
      <tr>
        <th>Date</th>
        <th>Title</th>
      </tr>
    </thead> -->
    <tbody>
      {% for post in collections.blog %}
      <tr>
        <td>{{ post.date | date: "MMMM dd, yyyy" }}</td>
        <td><a href="{{ post.url | url }}">{{ post.data.title }}</a></td>
      </tr>
      {% endfor %}
    </tbody>
  </table>
</div>