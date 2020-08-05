#include "animation.hpp"

#include <cmath>
#include <glm/glm.hpp>

#include "osksdl.hpp"

Animation::Animation(SDL_Renderer* renderer, SpriteSheet* sprite_sheet,
    std::vector<int> sprite_indices)
  : renderer_(renderer)
  , sprite_sheet_(sprite_sheet)
  , sprite_indices_(sprite_indices)
{
  num_sprites_ = sprite_indices_.size();
}

Animation::Animation(SDL_Renderer* renderer, SpriteSheet* sprite_sheet, int num_sprites)
  : renderer_(renderer)
  , sprite_sheet_(sprite_sheet)
  , num_sprites_(num_sprites)
  , sprite_indices_(num_sprites)
{
  for (int i = 0; i < num_sprites; ++i)
  {
    sprite_indices_[i] = i;
  }
}

void Animation::Render(const SDL_Rect* rect, SDL_RendererFlip flip) const
{
  const double cur_time = fmod(osksdl::GetElapsedTime(), animation_time_);
  const int cur_frame = glm::min(
      num_sprites_*(cur_time/animation_time_),
      static_cast<double>(num_sprites_-1));
  
  const int frame_idx = sprite_indices_[cur_frame];
  const SDL_Rect source_rect = sprite_sheet_->GetSpriteRect(frame_idx);

  SDL_RenderCopyEx(renderer_, sprite_sheet_->GetTexture(),
      &source_rect, rect, 0.0, nullptr, flip);
}

void Animation::SetAnimationTime(double t)
{
  animation_time_ = t;
}
