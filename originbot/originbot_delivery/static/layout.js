/* 吸顶/吸底布局的高度同步。
 *
 * CSS 里用两个自定义属性做错位，避免几层吸附元素互相压住：
 *   --header-h     标题栏高度   → 参数页工具条的 top
 *                              （工具条也吸顶，不给它让位就会压在标题栏下面）
 *   --statusbar-h  状态栏高度   → 参数页分节操作条的 bottom
 *                              （操作条也吸底，不让位就会盖住吸底的状态栏）
 *
 * 两个高度都不是固定值：窄窗口下标题栏会换行、状态栏高度也随字体变化，
 * 所以在这里实测后写回 CSS 变量，并跟随尺寸变化更新。
 */
(function () {
  const root = document.documentElement;
  const last = {};

  function setVar(name, value) {
    // 只在变化时写，避免 ResizeObserver 因写入而反复回调
    if (last[name] === value) return;
    last[name] = value;
    root.style.setProperty(name, value);
  }

  function measure() {
    const header = document.querySelector("header");
    const status = document.querySelector(".status-panel");
    setVar("--header-h", header ? header.offsetHeight + "px" : "0px");
    setVar("--statusbar-h", status ? status.offsetHeight + "px" : "0px");
  }

  function start() {
    measure();
    if (window.ResizeObserver) {
      const observer = new ResizeObserver(measure);
      document.querySelectorAll("header, .status-panel").forEach(function (el) {
        observer.observe(el);
      });
    }
    // 字体加载完高度可能还会变，补测一次
    window.addEventListener("load", measure);
    window.addEventListener("resize", measure);
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", start);
  } else {
    start();
  }
})();
