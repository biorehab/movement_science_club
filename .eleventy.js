module.exports = function(eleventyConfig) {
  // Tell Eleventy to copy the assets folder to `_site`
  eleventyConfig.addPassthroughCopy("src/assets");

  // Define the "posts" collection that will contain all blogposts
  eleventyConfig.addCollection("movsciposts", function(collectionApi) {
    return collectionApi.getFilteredByGlob("src/posts/*.md").reverse();
  });

  eleventyConfig.addFilter("date", function (date, format) {
    const { DateTime } = require("luxon");
    return DateTime.fromJSDate(date).toFormat(format);
  });

  // Tell Eleventy where layouts are
  return {
    dir: {
      input: "src",
      output: "_site"
    },
    pathPrefix: "/movement_science_club/"
  };
};

