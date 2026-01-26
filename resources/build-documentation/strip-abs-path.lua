-- to strip the leading slash. This converts them back to relative paths, which forces Pandoc to use your --resource-path.
function Image (img)
  -- If path starts with /, remove the first character
  if img.src:sub(1,1) == "/" then
    img.src = img.src:sub(2)
    print("[DEBUG] Path was absolute. Converted to " .. img.src)
  end
  return img
end
